#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include "perception_nodelet.h"

PLUGINLIB_EXPORT_CLASS(fusion_nodelet::PerceptionNodelet, nodelet::Nodelet)
namespace fusion_nodelet{
PerceptionNodelet::PerceptionNodelet(){}
PerceptionNodelet::~PerceptionNodelet(){}

void PerceptionNodelet::onInit(){
    ROS_INFO("PerceptionNodelet is OK for test!!!");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();
    
    //移除离群点
    std::string outlier_removal_method = private_nh.param<std::string>("outlier_removal_method", "STATISTICAL");
    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = private_nh.param<int>("statistical_mean_k", 15);
      double stddev_mul_thresh = private_nh.param<double>("statistical_stddev", 1);

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_remove = sor;
    }
    
    float h_max = 1.95, h_min = 0.1;
    float h_lidar =private_nh.param<double>("h_lidar", 0.75);
    cluster_.reset(new Cluster(h_max, h_min, h_lidar));
    
    std::string cloud_topic = private_nh.param<std::string>("perception_cloud_topic", "/apollo/sensor/velodyne16/compensator/PointCloud2");
    static auto sub = nh.subscribe(cloud_topic, 10, &PerceptionNodelet::cloud_back, this);
    
    std::string cluster_topic = private_nh.param<std::string>("cluster_topic", "clustering_cloud");
    pub = nh.advertise<sensor_msgs::PointCloud2>(cluster_topic, 1);
    pub_ele = nh.advertise<sensor_msgs::PointCloud2>("elevat_cloud", 1);

    pub_box = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ROS_INFO_STREAM("PerceptionNodelet::onInit() ended!");
}
void PerceptionNodelet::cloud_back(pcl::PointCloud<PointT>::Ptr src_cloud){
    ROS_INFO_STREAM("------------------");
    static int frame;
    // ROS_INFO_STREAM("PerceptionNodelet::cloud_back frame_id: "<<frame);
    frame++;
    ros::Time begin_t = ros::Time::now();
    // if(frame > 3) return;
    if(src_cloud->empty()) return;
    auto cloud_header = src_cloud->header;
    // RemoveCloudByHeight(src_cloud);
    
    //cluster points
    // RemoveOutlierPoints(src_cloud);
    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    cluster_->Segment(src_cloud, clusters, filtered);

    filtered->header = cloud_header;
    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    //移除分割聚类后每一簇点云的离群点
    // for(int i = 0; i < clusters.size(); ++i){
    //     int j = clusters[i]->size();
    //     RemoveOutlierPoints(clusters[i]);
    //     // ROS_INFO_STREAM("outlier remove ratio:"<<clusters[i]->size()*100.0/j);
    // }

    //output cluster points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB tempt;
    for(int i = 0;i < clusters.size(); ++i)
    {
       for(int j = 0; j < clusters[i]->size(); ++j)
       {
           tempt.x = clusters[i]->points[j].x;
           tempt.y = clusters[i]->points[j].y;
           tempt.z = clusters[i]->points[j].z;
           tempt.r = (500*(i+1))%255;
           tempt.g = (125*(i+1))%255;
           tempt.b = (200*(i+1))%255;
           color_cloud->push_back(tempt);
       }
    }
    color_cloud->header = cloud_header;
    color_cloud->width = color_cloud->size();
    color_cloud->height = 1;
    color_cloud->is_dense = false;
    //output end

    //building box
    std::vector<pcl::PointCloud<PointT>::Ptr> boxs_points;
    cluster_->MinBoxPoints1(clusters, &boxs_points);
    for(int i = 0; i < boxs_points.size(); ++i){
       if(boxs_points[i]->size() == 0) continue;
       boxs_points[i]->header.frame_id = cloud_header.frame_id;
    }
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = cloud_header.frame_id;
    cluster_->BuildBoxs(boxs_points, line_list);

    ros::Time end_t = ros::Time::now();
    ROS_INFO_STREAM("process per frame using:"<<(end_t-begin_t).toNSec()/1e6);
    // ROS_INFO_STREAM("frame total:"<<double((end_t-begin_t).toSec())*1000);

    pub.publish(color_cloud);
    pub_ele.publish(filtered);
    pub_box.publish(line_list);
}
void PerceptionNodelet::RemoveCloudByHeight(pcl::PointCloud<PointT>::Ptr& cloud){
    pcl::PointCloud<PointT>::Ptr filter(new pcl::PointCloud<PointT>);
    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filter->points),
        [&](PointT& p){
            if(p.x < 1.7 && p.x > -1.4 && p.y < 0.7 && p.y > -1.0)
               return false;
            else return p.z + lidar_height > 1.95? false : true; // || p.z + lidar_height < 0.8 
        }
    );
    cloud = filter;
}
void PerceptionNodelet::RemoveOutlierPoints(pcl::PointCloud<PointT>::Ptr& cloud){
    pcl::PointCloud<PointT>::Ptr filter(new pcl::PointCloud<PointT>);
    outlier_remove->setInputCloud(cloud);
    outlier_remove->filter(*filter);
    cloud = filter;
}
}//end namespace fusion_nodelet