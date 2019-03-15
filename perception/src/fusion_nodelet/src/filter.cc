// #include <float.h>
#include "filter.h"
#include <ros/ros.h>
namespace fusion_nodelet{
//class Cell
Cell::Cell():height_(FLT_MAX),is_ground_(false){

}
float Cell::sensor_h = 0;
void Cell::SetSensorHeight(float h){ Cell::sensor_h = h;}
void Cell::UpdateHeight(float z){
    if(z < 0 && z >= -sensor_h && z < height_) {height_ = z;} // && z < height_
    else if(z > 0){height_ = sensor_h;}
    else if(z < -sensor_h){height_ = -sensor_h - 0.2;} 
}
//class GaussBlur
void GaussBlur::GaussKernel(std::vector<double>& kernel, int samples, double sigma){
    //ROS_INFO("GaussBlur::GaussKernel++++++");
    double mean = samples/2;
    double sum = 0.0;
    for(size_t i = 0; i < samples; ++i){
        kernel[i] = exp(-0.5*(pow((i - mean)/sigma, 2.0)))/(2*M_PI*sigma*sigma);
        sum +=  kernel[i];
    }
    //Normalize the kernel
    for(size_t i = 0; i < samples; ++i)
        kernel[i] /= sum;
    //ROS_INFO("GaussBlur::GaussKernel------");
}
void GaussBlur::GaussSmooth(std::array<Cell, bins_>& channel, int samples, double sigma){
    //ROS_INFO("GaussBlur::GaussSmooth++++++");
    std::vector<double> kernel(samples);
    GaussKernel(kernel, samples, sigma);
    int sample_side = samples/2;
    for(size_t i = 0; i < channel.size(); ++i){
        double smooth = 0;
        for(size_t j = i - sample_side; j <= i + sample_side; ++j){
            if(j >= 0 && j < channel.size()){
                smooth += kernel[j - i + sample_side] * channel[j].GetHeight();
            }
        }
        channel[i].UpdateSmooth(smooth);
    }
    //ROS_INFO("GaussBlur::GaussSmooth------");
}
//class Filter
ClusterFilter::ClusterFilter(float h_lidar, float r_max,  float car_length, float car_width,  
                             float r_min,   float h_diff, float radius,     float h_max):  //一般默认的值即可
                             car_length(car_length),
                             car_width(car_width),
                             h_lidar(h_lidar),
                             radius(radius),
                             h_max(h_max),
                             r_max_(r_max),//地面去除的范围
                             r_min_(r_min),
                             h_thresh(h_diff){

    ROS_INFO_STREAM("ClusterFilter Constructor------------");
    init();
}
ClusterFilter::~ClusterFilter(){

}
void ClusterFilter::init(){
    ROS_INFO_STREAM("R_MAX: "<<r_max_);
    Cell::SetSensorHeight(h_lidar);
    gauss_.reset(new GaussBlur());
    //降采样
    double downsample_resolution = 0.09;
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;      
    //去除离群点
    int mean_k = 10;
    double stddev_thresh = 1.0;
    pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
    sor->setMeanK(mean_k);
    sor->setStddevMulThresh(stddev_thresh);
    outlier_remove = sor;
}

void ClusterFilter::Filter(const PointCloud::Ptr& src_cloud, PointCloud::Ptr& filtered, float& sensor_h){
    // ros::Time begin_t = ros::Time::now();

    PointCloud::Ptr temp(new PointCloud);
    DownSample(src_cloud, temp);

    // ros::Time sam_time = ros::Time::now();
    // ROS_INFO_STREAM("downsample  time:"<<double((sam_time-begin_t).toSec())*1000);

    // OutlierRemove(temp);

    // ros::Time outlier_time = ros::Time::now();
    // ROS_INFO_STREAM("outlier_re  time:"<<double((outlier_time - sam_time).toSec())*1000);

    RemoveGround(temp, sensor_h);

    // ros::Time ground_time = ros::Time::now();
    // ROS_INFO_STREAM("remove_groundime:"<<double((ground_time - outlier_time).toSec())*1000);

    filtered = temp;
}

void ClusterFilter::DownSample(const PointCloud::Ptr& src_cloud, PointCloud::Ptr& filtered){
    //缩小使用的点云范围
    // ros::Time begin_t = ros::Time::now();
    PointCloud::Ptr cloud(new PointCloud);
    std::copy_if(src_cloud->begin(), src_cloud->end(), std::back_inserter(cloud->points),
        [&](PointT& p){
            if(p.x < car_length/2 && p.x > -car_length/2 && p.y < car_width/2 && p.y > -car_width/2)
               return false;
            // else if(p.getVector3fMap().norm() > radius) //50m范围内  getVector3fMap坑死个人啊
            //     return false;
            else return p.z+h_lidar > h_max? false : true; // || p.z + lidar_height < 0.8 
        }
    );

    // ros::Time range_time = ros::Time::now();
    // ROS_INFO_STREAM("range_      time:"<<double((range_time-begin_t).toSec())*1000);
    
    if(!downsample_filter) {
      return ;
    }
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    // ROS_INFO_STREAM("DownSample: "<<filtered->size()/double(cloud->size())*100);
    // filtered = cloud;

    // ros::Time sam_time = ros::Time::now();
    // ROS_INFO_STREAM("downsample  time:"<<double((sam_time-range_time).toSec())*1000);
}

void ClusterFilter::OutlierRemove(PointCloud::Ptr& cloud){
    pcl::PointCloud<PointT>::Ptr filter(new pcl::PointCloud<PointT>);
    outlier_remove->setInputCloud(cloud);
    outlier_remove->filter(*filter);
    cloud = filter;
}

void ClusterFilter::RemoveGround(pcl::PointCloud<PointT>::Ptr& cloud, float& sensor_h){
    //filter cloud between r_min_~r_max_
    //ROS_INFO("ClusterFilter::RemoveGround++++++++++++++++++++++++++++++++++++++++++++");
    pcl::PointCloud<PointT>::Ptr filter(new pcl::PointCloud<PointT>);
    float dist;
    for(size_t i = 0; i < cloud->points.size(); ++i){
        auto &pt = cloud->points[i];
        dist = sqrt(pt.x * pt.x + pt.y * pt.y);
        if(pt.x < 0 && sensor_h > pt.z) sensor_h = pt.z;  // std::abs(pt.y)< 0.1 &&
        if(dist <= r_min_ || dist >= r_max_) continue;
        if(std::abs(pt.y) > 15) continue;
        filter->push_back(std::move(pt));
    }
    sensor_h = -sensor_h;
    h_lidar = sensor_h;
    Cell::SetSensorHeight(h_lidar);

    std::array<std::array<Cell, bins_>, channels_> polar_data{};
    MapCloudToPolarGrid(filter, polar_data);
    for(size_t i = 0; i < channels_; ++i){
        gauss_->GaussSmooth(polar_data[i], 3, 1);
        ComputeHDiffWithBorderCell(polar_data[i]);
        for(size_t j = 0; j < bins_; ++j){
            if(polar_data[i][j].GetSmooth() < 0 &&
                    polar_data[i][j].GetHDiff() < h_thresh){
                polar_data[i][j].UpdateGround();
            }
            else if(polar_data[i][j].GetHeight() < 0 &&
                    polar_data[i][j].GetHDiff() < h_thresh){
                polar_data[i][j].UpdateGround();
            }
            // ROS_INFO_STREAM("polar_data[i][j].num_pts:"<<polar_data[i][j].num_pts);
            if(polar_data[i][j].num_pts < 3)  polar_data[i][j].UpdateGround();
        }
    }
    MedianFilter(polar_data);
    OutlierFilter(polar_data);
    cloud->points.clear();
    for(size_t i = 0; i < filter->size(); ++i){
        auto pt = filter->points[i];
        int ch_i, bin_i;
        GetCellIndexByPoint(pt.x, pt.y, ch_i, bin_i);
        if(ch_i < 0 || ch_i > channels_ || bin_i <0 || bin_i > bins_) continue;
        if(polar_data[ch_i][bin_i].Ground()){
            float h_ground = polar_data[ch_i][bin_i].GetHeight();
            if(pt.z > (h_ground + 0.15)) cloud->points.push_back(std::move(pt));
        }
        else{
            cloud->points.push_back(std::move(pt));
        }
    }
    //ROS_INFO("ClusterFilter::RemoveGround--------------------------------------");
}
void ClusterFilter::MapCloudToPolarGrid(const pcl::PointCloud<PointT>::Ptr& cloud, 
        std::array<std::array<Cell, bins_>, channels_>& polar_data){
    //ROS_INFO("ClusterFilter::MapCloudToPolarGrid++++++");
    int ch_i, bin_i;
    for(size_t i = 0; i < cloud->size(); ++i){
        auto &pt = cloud->points[i];
        GetCellIndexByPoint(pt.x, pt.y, ch_i, bin_i);
        if(ch_i < 0 || ch_i > channels_ || bin_i <0 || bin_i > bins_) continue;
        polar_data[ch_i][bin_i].UpdateHeight(pt.z);
        polar_data[ch_i][bin_i].num_pts++;
    }
    //ROS_INFO("ClusterFilter::MapCloudToPolarGrid-----");
}
void ClusterFilter::GetCellIndexByPoint(float x, float y, int &ch_i, int &bin_i){
    // //ROS_INFO("ClusterFilter::GetCellIndexByPoint++++++");
    float dist = sqrt(x*x + y*y);
    float ch_p = (atan2(y, x) + M_PI)/(2 * M_PI);
    float bin_p = (dist - r_min_)/(r_max_ - r_min_);
    ch_i = floor(ch_p * channels_);
    bin_i = floor(bin_p * bins_);
    // //ROS_INFO("ClusterFilter::GetCellIndexByPoint-----");
}
void ClusterFilter::ComputeHDiffWithBorderCell(std::array<Cell, bins_>& channel){
    //ROS_INFO("ClusterFilter::ComputeHDiffWithBorderCell++++++");
    for(size_t i = 0; i < channel.size(); ++i){
        if( i == 0 ){
            float h_diff = channel[i].GetHeight() - channel[i+1].GetHeight();
            channel[i].UpdateHDiff(h_diff);
        }
        else if(i == channel.size() - 1){
            float h_diff = channel[i].GetHeight() - channel[i - 1].GetHeight();
            channel[i].UpdateHDiff(h_diff);
        }
        else{
            float diff_pre = channel[i].GetHeight() - channel[i-1].GetHeight();
            float diff_pos = channel[i].GetHeight() - channel[i+1].GetHeight();
            diff_pre > diff_pos ? channel[i].UpdateHDiff(diff_pre) : channel[i].UpdateHDiff(diff_pos);
        }
    }
    //ROS_INFO("ClusterFilter::ComputeHDiffWithBorderCell-----");
}
void ClusterFilter::MedianFilter(std::array<std::array<Cell, bins_>, channels_>& polar_data){
    //ROS_INFO("ClusterFilter::MedianFilter++++++");
    std::vector<float> sur;
    for(int i = 1; i < channels_-1; i++){
        for(int j = 1; j < bins_-1; j++){
            if(!polar_data[i][j].Ground()){
                if(polar_data[i][j+1].Ground()&&
                   polar_data[i][j-1].Ground()&&
                   polar_data[i+1][j].Ground()&&
                   polar_data[i-1][j].Ground()){
                     sur = {polar_data[i][j+1].GetHeight(),
                            polar_data[i][j-1].GetHeight(),
                            polar_data[i+1][j].GetHeight(),
                            polar_data[i-1][j].GetHeight()};
                    sort(sur.begin(), sur.end());
                    float m1 = sur[1]; float m2 = sur[2];
                    float median = (m1+m2)/2;
                    polar_data[i][j].UpdateHeight(median);
                    polar_data[i][j].UpdateGround();
                }
            }
        }
    }
    //ROS_INFO("ClusterFilter::MedianFilter------");
}
void ClusterFilter::OutlierFilter(std::array<std::array<Cell, bins_>, channels_>& polar_data){
    for(int i = 1; i < polar_data.size() - 1; i++) {
        for (int j = 1; j < polar_data[0].size() - 2; j++) {
            if(polar_data[i][j].Ground()&&
               polar_data[i][j+1].Ground()&&
               polar_data[i][j-1].Ground()&&
               polar_data[i][j+2].Ground()){
                float height1 = polar_data[i][j-1].GetHeight();
                float height2 = polar_data[i][j].GetHeight();
                float height3 = polar_data[i][j+1].GetHeight();
                float height4 = polar_data[i][j+2].GetHeight();
                if(height1 != -h_lidar && height2 == -h_lidar && height3 != -h_lidar){
                    float newH = (height1 + height3)/2;
                    polar_data[i][j].UpdateHeight(newH);
                    polar_data[i][j].UpdateGround();
                }
                else if(height1 != -h_lidar && height2 == -h_lidar && height3 == -h_lidar && height4 != -h_lidar){
                    float newH = (height1 + height4)/2;
                    polar_data[i][j].UpdateHeight(newH);
                    polar_data[i][j].UpdateGround();
                }
            }
        }
    }
}
}//namespace fusion_nodelet