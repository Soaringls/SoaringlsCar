#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("/scan",20, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/lslidar_calib",10);
    }

    void cloudCB(const sensor_msgs::LaserScanPtr& input)
    {
        static int pts_num = 1;
        std::ofstream file;
        file.open("data/lslidar.csv", std::ios_base::app);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointXYZ pt;
        cloud->points.reserve(input->ranges.size());
        for(size_t i = 0; i < input->ranges.size(); ++i) {
            double r = input->ranges[i];
            if(std::isfinite(r)) {
                float a = input->angle_min + i * input->angle_increment;
                pt.x = input->ranges[i] * cosf(a);
                pt.y = input->ranges[i] * sinf(a);
                if(std::sqrt(pt.x*pt.x + pt.y*pt.y) < 1)  continue;
                pt.z = 0;
                cloud->points.push_back(pt);

                if(pts_num == 1)
                    file<<"x"<<"\t"<<"y"<<"\t"<<"z\n";
                file<<pt.x<<"\t"<<pt.y<<"\t"<<pt.z<<"\n";
                pts_num++;
            }
        }
        cloud->header.frame_id = input->header.frame_id;
        cloud->header.stamp = input->header.stamp.toSec();
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = false;
        
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        pcl_pub.publish(output);
        
        file.close();     
        ROS_INFO_STREAM("point num is:"<<pts_num);   
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "lslidar_trans");

    cloudHandler handler;

    ros::spin();

    return 0;
}