#ifndef PERCETION_NODELET_H_
#define PERCETION_NODELET_H_
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "clustering.h"

namespace fusion_nodelet{

class PerceptionNodelet : public nodelet::Nodelet{
public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    PerceptionNodelet();
    virtual ~PerceptionNodelet();

    virtual void onInit();
private:
    void cloud_back(pcl::PointCloud<PointT>::Ptr src_cloud);
    void RemoveCloudByHeight(pcl::PointCloud<PointT>::Ptr& cloud);
    void RemoveOutlierPoints(pcl::PointCloud<PointT>::Ptr& cloud);
    //ros variables
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher pub;
    ros::Publisher pub_box;
    ros::Publisher pub_ele;

    pcl::Filter<PointT>::Ptr outlier_remove;
    //variables
    std::unique_ptr<Cluster> cluster_;
    float lidar_height;
};
}//end namespace fusion_nodelet
#endif