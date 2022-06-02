#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>


void downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud, float voxel_size)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new  pcl::PointCloud<pcl::PointXYZ>);


    std::vector<int> indices;

    // Create the filtering object
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (voxel_size, voxel_size, voxel_size);
    sor.filter (*cloud_downsampled);


    pcl::removeNaNFromPointCloud (*cloud_downsampled, *out_cloud, indices);


    return ;
}

void floor_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_removed(new  pcl::PointCloud<pcl::PointXYZ>);

    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    // add condition to remove floor
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.05)));
    // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.1)));

    // removing a little of upper points
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.4)));
    // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 1.2)));


    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud_in);
    condrem.setKeepOrganized(true);

    // apply filter
    condrem.filter (*floor_removed);


    std::vector<int> indices;

    pcl::removeNaNFromPointCloud (*floor_removed, *cloud_out, indices);


    return ;

}


Eigen::Matrix4f pair_alignment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    
    gicp.setInputSource(cloud_in);
    gicp.setInputTarget(cloud_out);
    gicp.setMaximumOptimizerIterations(50);

    pcl::PointCloud<pcl::PointXYZ> Final;

    gicp.align(Final);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(); // Transformation computed between scans

    transform = gicp.getFinalTransformation();

    return transform;
}
