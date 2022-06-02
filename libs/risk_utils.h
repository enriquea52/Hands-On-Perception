float risk_calc( Eigen::Vector3d x_0,   Eigen::Vector3d x_1,   Eigen::Vector3d x_2);

class centroid {
    public:
        float x, y, z, risk;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_addr;
        centroid(){};
        ~centroid(){};
};