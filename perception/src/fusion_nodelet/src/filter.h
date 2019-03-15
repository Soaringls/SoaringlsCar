#ifndef FILTER_H_
#define FILTER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace fusion_nodelet{
//class Cell
class Cell{
public: 
    Cell();
    static void SetSensorHeight(float h);
    void UpdateHeight(float h);
    float GetHeight(){return height_;}
    void UpdateSmooth(float s){smooth_ = s;}
    float GetSmooth(){return smooth_;}
    void UpdateHDiff(float d){h_diff_ = d;}
    float GetHDiff(){return h_diff_;}
    void UpdateGround(){is_ground_ = true; h_ground_ = height_;}
    bool Ground(){return is_ground_;}
    int num_pts;  //落入每个cell中点的数目
private:
    static float sensor_h;
    float smooth_;
    float height_, h_ground_, h_diff_;
    bool is_ground_; 
};

static constexpr int channels_ = 800;
static constexpr int bins_     = 220;

//class GaussBlur
class GaussBlur{
public:
    // friend class Filter;
    void GaussKernel(std::vector<double>& kernel, int samples, double sigma);
    void GaussSmooth(std::array<Cell, bins_>& bin_, int samples, double sigma);
};
//class ClusterFilter
class ClusterFilter{
public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    ClusterFilter(float h_lidar = 0.75, float r_max  = 30,  float car_length = 3, float car_width = 2, 
                  float r_min   = 0.5,  float h_thresh = 0.15,float radius     = 50, float h_max    = 3 );
    ~ClusterFilter();

    void init();
    void Filter(const PointCloud::Ptr& src_cloud, PointCloud::Ptr& filtered, float& sensor_h);
    void DownSample(const PointCloud::Ptr& cloud, PointCloud::Ptr& filtered);
    void OutlierRemove(PointCloud::Ptr& cloud);
    
    void RemoveGround(PointCloud::Ptr& cloud, float& sensor_h);
    void MapCloudToPolarGrid(const pcl::PointCloud<PointT>::Ptr& cloud, 
        std::array<std::array<Cell, bins_>, channels_>& polar_data);
    void GetCellIndexByPoint(float x, float, int &ch_i, int &bin_i);
    void ComputeHDiffWithBorderCell(std::array<Cell, bins_>& channel);
    void MedianFilter(std::array<std::array<Cell, bins_>, channels_>& polar_data);
    void OutlierFilter(std::array<std::array<Cell, bins_>, channels_>& polar_data);
private:
    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Filter<PointT>::Ptr outlier_remove;
    
    float h_lidar;
    float car_length, car_width;
    float r_max_, r_min_;//滤出地面的距离范围

    
    float radius; //使用点云的范围
    float h_max;  //z值大于此值的点将被滤出
    float h_thresh;

    std::unique_ptr<GaussBlur> gauss_;
};
}//fusion_nodelet
#endif