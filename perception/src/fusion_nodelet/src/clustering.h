#ifndef CLUSTERING_H_
#define CLUSTERING_H_
#include <vector>
#include "filter.h"
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
namespace fusion_nodelet{
class Cluster{
public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    static constexpr float resolution = 8; //单位cm,即几个厘米为一个格子
    static constexpr float roi_m_     = 80;
    static constexpr int num_grids_   = 100*roi_m_/resolution;

    using Grid = std::array<std::array<int, num_grids_>, num_grids_>;

    Cluster(float h_max = 2, float h_min = 0.15, float h_lidar = 0.75, int min_pts_num = 3);
    ~Cluster();
    void Segment(const PointCloud::Ptr& src_cloud, std::vector<PointCloud::Ptr>& clusters, PointCloud::Ptr& cloud);//, std::vector<PointCloud>& objects);
    void MinBoxPoints1(std::vector<PointCloud::Ptr>& objects, std::vector<PointCloud::Ptr>* obstacle);
    void BuildBoxs(std::vector<PointCloud::Ptr>& objects, visualization_msgs::Marker& line_list);
private:
    void MapCloudToGrid(PointCloud::Ptr& cloud, Grid& grid);

    void SerializeObjects(Grid& grid, int& num);
    void Search(Grid& grid, int num, int row, int col);

    //box build
    void SegmentCloud(PointCloud::Ptr& cloud, Grid& roi_grid, int num_raw_object, std::vector<PointCloud::Ptr>* clusters);
    void ObjectsPoints(const PointCloud::Ptr& cloud, Grid& roi_grid, std::vector<PointCloud::Ptr>* objects);
    void CutObjectByZ(std::vector<PointCloud::Ptr>* objects);

    void BorderMaxMinXY(const PointCloud::Ptr& cloud, Eigen::Vector3f* min_point, Eigen::Vector3f* max_point);
    
    bool FilterNormalObject(std::vector<Eigen::Vector3f>& box, float h, int num);
    //variables
    static constexpr int kernel = 3;
    int min_pts_num;
    float h_lidar;
    float h_min;
    float h_max;

    std::unique_ptr<ClusterFilter> filter;
};
}//end namespace fusion_nodelet
#endif