#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <fstream>
#include <sstream>
#include <string>
// #include <stringstream>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ndt.h>
#include <pclomp/ndt_omp.h>

namespace fusion_nodelet{
class FusionCloudNodelet : public nodelet::Nodelet{
public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    FusionCloudNodelet(){}
    virtual ~FusionCloudNodelet(){}
    
    virtual void onInit(){
        ROS_INFO("FusionCloudNodelet is OK for test!!!");
        nh = getNodeHandle();
        private_nh = getPrivateNodeHandle();

        initialize_params();
        GetTransforms();
    }
private:
    void initialize_params(){
        trans_file = private_nh.param<std::string>("trans_file_path", "${find fusion_nodelet}/config/calib_lidar.txt");

        topic_l = private_nh.param<std::string>("topic_left", "/velodyne_points");
        topic_r = private_nh.param<std::string>("topic_right", "/velodyne_points2");
        topic_pub = private_nh.param<std::string>("fusion_cloud", "/apollo/sensor/velodyne16/compensator/PointCloud2");
        //ndt registration
        double ndt_resolution = 0.5;
        boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
        ndt->setTransformationEpsilon(0.01);
        ndt->setMaximumIterations(64);
        ndt->setResolution(ndt_resolution);
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        registration = ndt;

        sub_left.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, topic_l, 10));
        sub_right.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, topic_r, 10));
        sync.reset(new message_filters::Synchronizer<sync_policy>(sync_policy(10), *sub_left, *sub_right));
        sync->registerCallback(boost::bind(&FusionCloudNodelet::CloudBack, this, _1, _2));

        pub = nh.advertise<sensor_msgs::PointCloud2>(topic_pub, 10);
        ROS_INFO_STREAM("initialize_params is ok!!!");
    }
    void CloudBack(const sensor_msgs::PointCloud2ConstPtr& in_left, const sensor_msgs::PointCloud2ConstPtr& in_right){
        // ROS_INFO_STREAM("--------------------------------------------------");
        pcl::PointCloud<PointT>::Ptr cloud_left(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr trans_left(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_right(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr trans_right(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*in_left, *cloud_left);
        pcl::fromROSMsg(*in_right, *cloud_right);

        pcl::transformPointCloud(*cloud_left, *trans_left, trans_[0]);
        pcl::transformPointCloud(*cloud_right, *trans_right, trans_[1]);
        
        if(!ndt_flag){
            ndt_flag = true;
            Eigen::Matrix4f ndt_trans;
            ndt_trans.setIdentity();
            registration->setInputTarget(trans_left);
            registration->setInputSource(trans_right);
            pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>);
            registration->align(*aligned,ndt_trans);
            trans_right = aligned;
            ndt_trans = registration->getFinalTransformation();
            trans_[1] = ndt_trans*trans_[1];
            // trans_[1] = trans_[1] * ndt_trans;
            ROS_INFO_STREAM("ndt*tr:\n"<<ndt_trans*trans_[0]);//矩阵相乘不满足交换律
            ROS_INFO_STREAM("tr* ndt:\n"<<trans_[0] * ndt_trans);
        }

        trans_right->reserve(trans_left->points.size() + trans_right->points.size());

        for(std::size_t i = 0; i < trans_left->points.size(); ++i)
        {
            trans_right->push_back(std::move(trans_left->points[i]));
        }
        trans_right->width = trans_right->size();
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*trans_right, *output);
        
        pub.publish(output);
    }
    void GetTransforms(){
        // std::ifstream file(trans_file.c_str());
        // if(!file){
        //     ROS_ERROR("Failed to open the file which contains left and right trans pose!");
        //     return;
        // }
        // std::string line, temp;
        // while(getline(file, line)){
        //     std::istringstream sstring(line);
        //     // sstring>>temp;
        //     // if(!aotf(temp)) continue;
        //     while(sstring>>temp && atof(temp.c_str()))
        //         trans_<<temp;
        // }
        ROS_INFO_STREAM("pose_path: "<<trans_file);
        std::ifstream ifs(trans_file.c_str());
        if(!ifs.is_open())
        {
            ROS_ERROR("Failed to open the file which contains left and right trans pose!");
            return;
        }
        std::string flag;
        getline(ifs, flag);
        Eigen::Matrix4f trans_temp = Eigen::Matrix4f::Identity();
        while(!ifs.eof())
        {
            for(int j = 0; j < 4; ++j)
            {
                for(int k = 0; k < 4; ++k)
                {
                    ifs>>trans_temp(j,k);
                }
            }
            trans_.push_back(trans_temp);
            getline(ifs, flag);
            getline(ifs, flag);
        }
    }
private:
    std::string topic_l, topic_r, topic_pub;
    std::string trans_file;
    std::vector<Eigen::Matrix4f> trans_;
    // Eigen::Matrix<float, 8, 4> trans_;

    bool ndt_flag;
    pcl::Registration<PointT, PointT>::Ptr registration;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_left;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_right;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync_policy;
    std::unique_ptr<message_filters::Synchronizer<sync_policy>> sync;
    ros::Publisher pub;
};
}//namespace fusion_nodelet
PLUGINLIB_EXPORT_CLASS(fusion_nodelet::FusionCloudNodelet, nodelet::Nodelet)