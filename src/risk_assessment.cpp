// STL libraries
#include <iostream>
#include <vector>

//ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
//////////////////////////////////////////
#include <visualization_msgs/Marker.h>  //
#include <std_msgs/ColorRGBA.h>         //
#include <geometry_msgs/Point.h>        //
//////////////////////////////////////////
#include "perception/emergency.h"

// PCL libraries
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

// Custom libraries
#include "../libs/pcl_utils.h"
#include "../libs/risk_utils.h"

//Eigen libraries
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <Eigen/Dense>

// OpenCV libraries
#include <opencv2/opencv.hpp>

class risk_assessment
{
public:
  
  risk_assessment(std::string pointcloud_topic, 
                  std::string risk_pointcloud, 
                  std::string closest_obs_arrow, 
                  std::string farther_obs_topic, 
                  std::string emergency_service)
  {
    // periodic timer
    timer_ = n_.createTimer(ros::Duration(1.0 / 10.0),&risk_assessment::risk_computation, this);

    //Topic you want to subscribe
    sub_ = n_.subscribe(pointcloud_topic, 1, &risk_assessment::callback, this);

    //Publishing resulting risk pointcloud
    pub_ = n_.advertise<sensor_msgs::PointCloud2>( risk_pointcloud, 1);
    
    //Resulting Repulsive Vector visualization
    vis_pub_rrp = n_.advertise<visualization_msgs::Marker>( closest_obs_arrow, 0 );

    //Better Route To Take (brtt) for the robot
    vis_pub_brtt = n_.advertise<visualization_msgs::Marker>( farther_obs_topic, 0 );

    // Emergency reactive action for the robot to take
    client = n_.serviceClient<perception::emergency>(emergency_service);

  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    // Pointcloud received from the sensor
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Downsampled pointcloud via voxelization
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Transformed cloud to the global reference frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // remove the floor
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_floor_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Receive pointclouud2 msg and convert it to PCl cloud
    pcl::fromROSMsg(*cloud, *current_cloud);

    // Using downsampling voxeling filter to reduce number of points in cloud
    downsampling(current_cloud, downsampled_cloud, 0.07);

    // Transform to odom frame
    tf::StampedTransform rs_to_odom;
    tf_listener.waitForTransform("odom", "realsense_link", ros::Time(0), ros::Duration(1));
    tf_listener.lookupTransform("odom", "realsense_link", ros::Time(0), rs_to_odom);
    pcl_ros::transformPointCloud(*downsampled_cloud, *transformed_cloud, rs_to_odom);

    // Remove the floor from the pointcloud
    floor_removal(transformed_cloud, no_floor_cloud);

    // Store the filtered pointcloud in a class vaariable to keep it updated
    filtered_cloud = *no_floor_cloud;

  }


  void risk_computation(const ros::TimerEvent&)
  {

    // Get transform from the corners of the bounding box
    tf::StampedTransform transform;

    if (is_init)
    {
      bb_listener.waitForTransform("odom", "bb8", ros::Time(0), ros::Duration(1));
      bb_listener.lookupTransform("odom", "bb8", ros::Time(0), transform);
      // Initialize the pose of the robot

      x_2 << transform.getOrigin().x(),
             transform.getOrigin().y(),
             transform.getOrigin().z();
      is_init = false;
    }
    else
    {
      bb_listener.waitForTransform("odom", "bb8", ros::Time(0), ros::Duration(1));
      bb_listener.lookupTransform("odom", "bb8", ros::Time(0), transform);
      
      // Update the past pose of the robot
      x_1 = x_2;

      //Update the current pose of the robot
      x_2 << transform.getOrigin().x(),
             transform.getOrigin().y(),
             transform.getOrigin().z();


      // Define Data structures for storing the risk factor and the normalization of the weights
      cv::Mat risk_factor(filtered_cloud.size(),1,CV_64F);
      cv::Mat normalized_weight_factors, mat_weights_colormap;

      // Clustering section ####################################################################
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

      *cloud_filtered = filtered_cloud;

      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud_filtered);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance (0.4); // 2cm
      ec.setMinClusterSize (40);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_filtered);
      ec.extract (cluster_indices);

      int j = 0;
      // Variables for risk post-processing
      float cum_risk; float max_risk = 0; int max_risk_idx = 0; float min_risk = 1000; int min_risk_idx = 0;

      if (cluster_indices.size() < 1){return;} // Safety measurement: In case there is not a cluster detected

      std::vector<centroid> cluster_centroids(cluster_indices.size());


      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {

        cum_risk = 0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CentroidPoint<pcl::PointXYZ> cluster_center;

        for (const auto& idx : it->indices)
        {
          cloud_cluster->push_back ((*cloud_filtered)[idx]); 
          cloud_cluster->width = cloud_cluster->size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;


          cluster_center.add((*cloud_filtered)[idx]);

            x_0 << (*cloud_filtered)[idx].x,
                   (*cloud_filtered)[idx].y,
                   (*cloud_filtered)[idx].z;

          // Appending risk to the risk_factor function for visualization of the risk on the whol pointcloud
          risk_factor.at<double>(idx) = risk_calc( x_0, x_1, x_2);


          // Accummulating the risk at the current pointcloud cluster
          cum_risk += risk_factor.at<double>(idx);

        }

        pcl::PointXYZ c;
        cluster_center.get(c);

        cluster_centroids[j].x = c.x;
        cluster_centroids[j].y = c.y;
        cluster_centroids[j].z = c.z;
        cluster_centroids[j].risk = cum_risk/cloud_cluster->size();
        cluster_centroids[j].cloud_addr = cloud_cluster;


        // find max and min risks and their respective indexes
        if(cluster_centroids[j].risk > max_risk){max_risk = cluster_centroids[j].risk; max_risk_idx = j;}
        if(cluster_centroids[j].risk < min_risk){min_risk = cluster_centroids[j].risk; min_risk_idx = j;}

        j++;

      }



      float largest_d = 0, largest_d_idx = -1, temp_d = 0;
      Eigen::Vector3d higher_risk_obstacle;
      Eigen::Vector3d lower_risk_obstacle;
      higher_risk_obstacle << cluster_centroids.at(max_risk_idx).x, cluster_centroids.at(max_risk_idx).y, cluster_centroids.at(max_risk_idx).z;



      for(int i = 0; i < cluster_centroids.size(); i++ )
      {

        lower_risk_obstacle << cluster_centroids.at(i).x, cluster_centroids.at(i).y, cluster_centroids.at(i).z;

        temp_d = ((higher_risk_obstacle  - x_2).cross(x_2 - lower_risk_obstacle)).norm() / (higher_risk_obstacle  - x_2).norm();

        if (temp_d > largest_d && i != max_risk_idx) 
        {
          largest_d = temp_d;
          largest_d_idx = i;
        }        
      }




      // Visualization code
      // Normalization of risk factors
      float min_weight = 0, max_weight = 1;
      normalized_weight_factors = (risk_factor - min_weight) / (max_weight - min_weight) * 255; 
      normalized_weight_factors.convertTo(normalized_weight_factors, CV_8UC1); // the weight_factors variable are the normalized risk values in a scale                               
      cv::applyColorMap(normalized_weight_factors, mat_weights_colormap,2);

      // Coloring the pointcloud according to the risk of each point
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr reconstructed_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (int p = 0; p < filtered_cloud.size(); p++) 
      {
          pcl::PointXYZRGB Point3d;               // pcl empty initialization PointCloud XYZ RGB
          Point3d.x = filtered_cloud.points[p].x; // Filling x, y, and z coordinates of the point cloud
          Point3d.y = filtered_cloud.points[p].y; // filtered point cloud
          Point3d.z = filtered_cloud.points[p].z;

          uint32_t rgb = (
              static_cast<uint32_t>((int) mat_weights_colormap.at<uchar>(p, 2)) << 16 | // Filling the color of each point depending
              static_cast<uint32_t>((int) mat_weights_colormap.at<uchar>(p, 1)) << 8 |  // on the risk value
              static_cast<uint32_t>((int) mat_weights_colormap.at<uchar>(p, 0)));
          Point3d.rgb = *reinterpret_cast<float *>(&rgb);
          reconstructed_point_cloud->points.push_back(Point3d); // PointCloud variables are normally used as Pointers, so you have to push the previous data to a pointer
      }

      // Vector visualization tool for checking the obstacle with higher risk
      vector_visualization(cluster_centroids.at(max_risk_idx).x, cluster_centroids.at(max_risk_idx).y, cluster_centroids.at(max_risk_idx).z, x_2(0), x_2(1), x_2(2), 1);
      // vector_visualization(x_2(0), x_2(1), x_2(2), cluster_centroids.at(min_risk_idx).x, cluster_centroids.at(min_risk_idx).y, cluster_centroids.at(min_risk_idx).z, 0);

      if (largest_d_idx >= 0)
      {
        vector_visualization(x_2(0), x_2(1), x_2(2), cluster_centroids.at(largest_d_idx).x, cluster_centroids.at(largest_d_idx).y, cluster_centroids.at(largest_d_idx).z, 0);

        if (cluster_centroids.at(max_risk_idx).risk > 0.95 && !isnan(cluster_centroids.at(max_risk_idx).risk))
        {
          if (min_risk_idx != max_risk_idx)
          {
            std::cout << "EMERGENCY!!! time to take  reactive control actions!!!!!" << std::endl;
            std::cout << "the excessive risk is: " << cluster_centroids.at(max_risk_idx).risk << " signal: 0" << std::endl;

            srv.request.signal = 0;
            srv.request.x = cluster_centroids.at(largest_d_idx).x;
            srv.request.y = cluster_centroids.at(largest_d_idx).y;
            srv.request.z = cluster_centroids.at(largest_d_idx).z;


            if (client.call(srv))
              {
                ROS_INFO("Obstacle succesfully Avoided: %ld", (long int)srv.response.avoided);
              }
            else
              {
                ROS_ERROR("Failed to call service emergency");
                return ;
              }

          }
        }
      }


      //Publishing the colored pointcloud representing the risk assessment
      sensor_msgs::PointCloud2 risk_cloud;
      pcl::toROSMsg(*reconstructed_point_cloud, risk_cloud);
      std::cout << "max_risk: " << cluster_centroids.at(max_risk_idx).risk << std::endl;
      //pcl::toROSMsg(*(cluster_centroids.at(max_risk_idx).cloud_addr), risk_cloud);
      risk_cloud.header.frame_id = "odom";
      pub_.publish(risk_cloud);

    }

  }




  void vector_visualization(float xi, float yi, float zi, float xf, float yf, float zf, int color, int id = 0)
  {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "odom";
      marker.header.stamp = ros::Time();
      marker.ns = "vector";
      marker.id = id;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;
      marker.scale.y = 0.03;
      marker.scale.z = 0.03;


      if(color == 0) // Selection of blue color
      {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0; 

        geometry_msgs::Point start;
        geometry_msgs::Point end;

        start.x = xi;
        start.y = yi;
        start.z = zi;

        end.x = xf;
        end.y = yf;
        end.z = zf;

        marker.points.push_back(start);
        marker.points.push_back(end);

        vis_pub_brtt.publish( marker );
      }
      if(color == 1) // Selection of red color
      {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; 

        geometry_msgs::Point start;
        geometry_msgs::Point end;

        start.x = xi;
        start.y = yi;
        start.z = zi;

        end.x = xf;
        end.y = yf;
        end.z = zf;

        marker.points.push_back(start);
        marker.points.push_back(end);

        vis_pub_rrp.publish( marker );
      }


  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Publisher vis_pub_rrp;
  ros::Publisher vis_pub_brtt;
  ros::ServiceClient client;
  perception::emergency srv;
  ros::Subscriber sub_;
  ros::Timer timer_;
  //cloud storage
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  pcl::PointCloud<pcl::PointXYZ> current_cloud;
  tf::TransformListener bb_listener;
  tf::TransformListener tf_listener;

  // past pose
  Eigen::Vector3d x_1;
  // current pose
  Eigen::Vector3d x_2;
  // current point
  Eigen::Vector3d x_0;
  // Resultant repulsive vector
  Eigen::Vector3d rrp;
  float rrp_mag;

  // check if it is initialized
  bool is_init = true;

};//End of class SubscribeAndPublish


int main(int argc, char **argv)
{

  std::cout << "RISK ASSESSMENT NODE IGNITED" << std::endl;

  //Initiate ROS risk assessment node
  ros::init(argc, argv, "Risk_Assessment_Node");

  std::string pointcloud_topic;
  std::string risk_pointcloud;
  std::string closest_obs_arrow;
  std::string farther_obs_topic;
  std::string emergency_service;

  // Read parameters from the parameter server
  ros::param::get("tbot_points_topic", pointcloud_topic);
  ros::param::get("pub_risk_pointcloud_topic", risk_pointcloud);
  ros::param::get("higher_risk_obs_topic", closest_obs_arrow);
  ros::param::get("lowest_risk_obs_topic", farther_obs_topic);
  ros::param::get("emergency_service", emergency_service);

  // Risk assessment node initialization
  risk_assessment node(pointcloud_topic, 
                  risk_pointcloud, 
                  closest_obs_arrow, 
                  farther_obs_topic, 
                  emergency_service);

  ros::spin();

  return 0;
}