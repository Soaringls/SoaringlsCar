#include "clustering.h"

namespace fusion_nodelet{
    

Cluster::Cluster(float h_max, float h_min, float h_lidar,int min_pts_num):
                min_pts_num(min_pts_num),
                h_lidar(h_lidar),
                h_max(h_max),
                h_min(h_min){
    ROS_INFO_STREAM("Cluster Constructor------------");
    ROS_INFO_STREAM("h_max:"<<h_max);
    ROS_INFO_STREAM("roi_m_:"<<roi_m_/2+1);
    filter.reset(new ClusterFilter(h_lidar, roi_m_/2+1));
}
Cluster::~Cluster(){

}
void Cluster::Segment(const PointCloud::Ptr& src_cloud, 
                             std::vector<PointCloud::Ptr>& clusters, PointCloud::Ptr& filtered)//, std::vector<PointCloud>& objects){
{
    // ros::Time begin_t = ros::Time::now();
    filter->Filter(src_cloud, filtered, h_lidar);
    // ROS_INFO_STREAM("sensor_h:"<<h_lidar);
    // ros::Time filter_time = ros::Time::now();
    // ROS_INFO_STREAM("------------------");
    // ROS_INFO_STREAM("filter time:"<<double((filter_time-begin_t).toSec())*1000);
    
    Grid roi_grid{}; //Grid roi_grid;将产生重大谬误的问题
    
    MapCloudToGrid(filtered, roi_grid);
    
    int num_objects{}; //int num_objects;并没有把变量num_objects初始化为0，将产生重大谬误的问题,num_objects将会无限增长
    SerializeObjects(roi_grid, num_objects);
    
    SegmentCloud(filtered, roi_grid, num_objects, &clusters);

    // ros::Time seg_time = ros::Time::now();
    // ROS_INFO_STREAM("seg    time:"<<double((seg_time-filter_time).toSec())*1000);
    // ROS_INFO_STREAM("filter&seg :"<<double((seg_time-begin_t).toSec())*1000);
}
void Cluster::MapCloudToGrid(PointCloud::Ptr& cloud, Grid& roi_grid){
    for(int i = 0; i < cloud->size(); ++i){
        if(cloud->points[i].z + h_lidar > h_max || 
           cloud->points[i].z + h_lidar < h_min ) {
            //    ROS_INFO_STREAM("cloud->points[i].z + h_lidar:"<<cloud->points[i].z + h_lidar);
               continue;  //不参与聚类
        }
        // if(cloud->points[i].z + h_lidar > h_max) {
        //     //    ROS_INFO_STREAM("cloud->points[i].z + h_lidar:"<<cloud->points[i].z + h_lidar);
        //        continue;  //不参与聚类
        // }
        float xC = cloud->points[i].x + roi_m_/2;
        float yC = cloud->points[i].y + roi_m_/2;
        // exclude outside roi points
        if(xC < 0 || xC >= roi_m_ || yC < 0 || yC >=roi_m_) continue;
        int xI = floor(num_grids_*xC/roi_m_);
        int yI = floor(num_grids_*yC/roi_m_);
        roi_grid[xI][yI] = -1;

        if(xI == 0)
        {
            if(yI == 0)
            {
                roi_grid[xI+1][yI] = -1;
                roi_grid[xI][yI+1] = -1;
                roi_grid[xI+1][yI+1] = -1;
            }
            else if(yI < num_grids_ - 1)
            {
                roi_grid[xI][yI-1] = -1;
                roi_grid[xI][yI+1] = -1;
                roi_grid[xI+1][yI-1] = -1;
                roi_grid[xI+1][yI] = -1;
                roi_grid[xI+1][yI+1] = -1;
            }
            else if(yI == num_grids_ - 1)
            {
                roi_grid[xI][yI-1] = -1;
                roi_grid[xI+1][yI-1] = -1;
                roi_grid[xI+1][yI] = -1;    
            }
        }
        else if(xI < num_grids_ - 1)
        {
            if(yI == 0)
            {
                roi_grid[xI-1][yI] = -1;
                roi_grid[xI-1][yI+1] = -1;
                roi_grid[xI][yI+1] = -1;
                roi_grid[xI+1][yI] = -1;
                roi_grid[xI+1][yI+1] = -1;                
            }
            else if(yI < num_grids_ - 1)
            {
                roi_grid[xI-1][yI-1] = -1;
                roi_grid[xI-1][yI] = -1;
                roi_grid[xI-1][yI+1] = -1;
                roi_grid[xI][yI-1] = -1;
                roi_grid[xI][yI+1] = -1;
                roi_grid[xI+1][yI-1] = -1;
                roi_grid[xI+1][yI] = -1;
                roi_grid[xI+1][yI+1] = -1;                  
            }
            else if(yI == num_grids_ - 1)
            {
                roi_grid[xI-1][yI-1] = -1;
                roi_grid[xI-1][yI] = -1;
                roi_grid[xI][yI-1] = -1;
                roi_grid[xI+1][yI-1] = -1;
                roi_grid[xI+1][yI] = -1;                 
            } 
        }
        else if(xI == num_grids_ - 1)
        {
            if(yI == 0)
            {
                roi_grid[xI-1][yI] = -1;
                roi_grid[xI-1][yI+1] = -1;
                roi_grid[xI][yI+1] = -1;
            }
            else if(yI < num_grids_ - 1)
            {
                roi_grid[xI-1][yI-1] = -1;
                roi_grid[xI-1][yI] = -1;
                roi_grid[xI-1][yI+1] = -1;
                roi_grid[xI][yI-1] = -1;
                roi_grid[xI][yI+1] = -1;
            }
            else if(yI == num_grids_ - 1)
            {
                roi_grid[xI-1][yI-1] = -1;
                roi_grid[xI-1][yI] = -1;
                roi_grid[xI][yI-1] = -1;    
            }            
        }
    }
}
void Cluster::SerializeObjects(Grid& roi_grid, int& num){
    for(int row = 0; row < num_grids_; ++row){
        for(int col = 0; col < num_grids_; ++col){
            if(roi_grid[row][col] == -1){
                num++;
                Search(roi_grid, num, row, col);
            }
        }
    }
}
void Cluster::Search(Grid& roi_grid, int num, int row, int col){
    roi_grid[row][col] = num;
    int mean = kernel / 2;
    for(int i = 0; i < kernel; ++i){
        int step_i = i - mean;
        if((row + step_i) < 0 || (row + step_i) >= num_grids_) continue;
        for(int j = 0; j < kernel; ++j){
            int step_j = j - mean;
            if((col + step_j) < 0 || (col + step_j) >= num_grids_) continue;
            if(roi_grid[row + step_i][col + step_j] == -1){
                Search(roi_grid, num, row+step_i, col+step_j);
            }
        }
    }
}
//build box
void Cluster::SegmentCloud(PointCloud::Ptr& cloud, Grid& roi_grid, 
                         int num_raw_object, 
                         std::vector<PointCloud::Ptr>* clusters){
    clusters->clear();
    clusters->reserve(num_raw_object);
    // objects->reserve(num_raw_object);
    // PointCloud::Ptr temp(new PointCloud);//切记不可这样，否则objects里面每个元素都指向同一个对象
    for(int i = 0; i < num_raw_object; ++i){
        PointCloud::Ptr temp(new PointCloud);
        clusters->push_back(temp);
    }    
    
    ObjectsPoints(cloud, roi_grid, clusters);
    //ros::Time begin = ros::Time::now();
    //for(size_t i = 0; i < clusters->size(); ++i){
    //    filter->OutlierRemove((*clusters)[i]);
    //}
    //ros::Time end = ros::Time::now();
    //ROS_INFO_STREAM("outlie time:"<<double((end-begin).toSec())*1000);
}
void Cluster::ObjectsPoints(const PointCloud::Ptr& cloud, Grid& roi_grid, std::vector<PointCloud::Ptr>* objects){
    for(size_t i = 0; i < cloud->size(); ++i){
        auto pt = cloud->points[i];
        float X = pt.x + roi_m_/2;
        float Y = pt.y + roi_m_/2;
        if(X < 0 || X >= roi_m_ || Y < 0 || Y >= roi_m_) continue;
        int row = std::floor(X * num_grids_/roi_m_);
        int col = std::floor(Y * num_grids_/roi_m_);
        int object_index = roi_grid[row][col];
        if( object_index != 0){
            (*objects)[object_index - 1]->push_back(pt);
        }
    }
}

void Cluster::CutObjectByZ(std::vector<PointCloud::Ptr>* objects){
    for(size_t i = 0; i < objects->size(); ++i){
        int num = (*objects)[i]->size();
    }
}

void Cluster::MinBoxPoints1(std::vector<PointCloud::Ptr>& objects, std::vector<PointCloud::Ptr>* obstacle){
    obstacle->clear();
    for(auto& cloud : objects){
        size_t pts_size = cloud->size();
        if(pts_size < 5u) { 
            continue;
        }
        //step1.find max_pt min_pt of the object
        Eigen::Vector3f max_border(std::numeric_limits<float>::lowest(),
                                std::numeric_limits<float>::lowest(),
                                std::numeric_limits<float>::lowest());
        Eigen::Vector3f min_border(std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max());
        BorderMaxMinXY(cloud, &min_border, &max_border);

        Eigen::Vector3f min_pt = Eigen::Vector3f::Zero();
        Eigen::Vector3f max_pt = Eigen::Vector3f::Zero();
        float min_tan = std::numeric_limits<float>::max();
        float max_tan = std::numeric_limits<float>::lowest();
        float max_z = std::numeric_limits<float>::lowest();
        
        for(size_t i = 0; i < pts_size; ++i){
            auto pt = cloud->points[i];
            if(pt.x == 0 || pt.z < 0) continue;
            //object which cross Y axle
            if(max_border.x() > 0 && min_border.x() < 0){
                //1,3象限远离Y轴的为max_pt
                if(pt.y/pt.x > 0 && pt.y/pt.x < min_tan){
                    min_tan = pt.y/pt.x;
                    max_pt.x() = pt.x;
                    max_pt.y() = pt.y;
                }
                //2,4象限远离Y轴的为min_pt
                else if(pt.y/pt.x < 0 && pt.y/pt.x > max_tan){
                    max_tan = pt.y/pt.x;
                    min_pt.x() = pt.x;
                    min_pt.y() = pt.y;
                }
            }
            else{//object which not cross Y axle
                if(pt.y/pt.x < min_tan){
                    min_tan = pt.y/pt.x;
                    min_pt.x() = pt.x;
                    min_pt.y() = pt.y;
                }
                if(pt.y/pt.x > max_tan){
                    max_tan = pt.y/pt.x;
                    max_pt.x() = pt.x;
                    max_pt.y() = pt.y;
                }
            }
            if(pt.z > max_z) max_z = pt.z;
        }

        //根据高度过滤分割对象
        if (max_z + h_lidar > 3 || max_z + h_lidar < 0.2) continue;

        for(size_t i = 0; i < pts_size; ++i){
            auto pt = cloud->points[i];
            if(pt.x == 0 || pt.z < 0) continue;
            //quadrant 1,3
            if(pt.y/pt.x > 0) {
                auto dist_near_x = (max_pt - pt.getVector3fMap()).norm();
                auto dist_near_y = (min_pt - pt.getVector3fMap()).norm();
                if(((max_pt - min_pt).norm() < dist_near_x) && 
                   (std::abs(pt.y - min_pt.y()) < resolution)){
                    min_pt = pt.getVector3fMap();
                }
                if(((max_pt - min_pt).norm() < dist_near_y) && 
                   (std::abs(pt.x - min_pt.x()) < resolution)){
                    // ROS_INFO_STREAM("---------------resolution: "<<resolution);
                    max_pt = pt.getVector3fMap();
                }
            }
            //quadrant 2,4
            else {
                auto dist_near_x = (min_pt - pt.getVector3fMap()).norm();
                auto dist_near_y = (max_pt - pt.getVector3fMap()).norm();
                if(((max_pt - min_pt).norm() < dist_near_x) && 
                   (std::abs(pt.y - min_pt.y()) < resolution)){
                    max_pt = pt.getVector3fMap();
                }
                if(((max_pt - min_pt).norm() < dist_near_y) && 
                   (std::abs(pt.x - min_pt.x()) < resolution)){
                    min_pt = pt.getVector3fMap();
                }
            }
        }
        //step2.find the corner point of the minBox
        Eigen::Vector3f min_to_max = Eigen::Vector3f::Zero();
        Eigen::Vector3f min_to_pt = Eigen::Vector3f::Zero();
        min_to_max = max_pt - min_pt;

        // //对细杆的检测
        // if(min_to_max.norm() < 0.05){
        //     min_pt.x() -= 0.05;
        //     min_pt.y() -= 0.05;
        // } 
        
        float max_dist = std::numeric_limits<float>::lowest();
        PointT corner_pt, infer_pt;
        for(const auto& pt : cloud->points){
            min_to_pt = pt.getVector3fMap() - min_pt;
            // auto flag = min_to_max.x()*min_to_pt.y() - min_to_pt.x()*min_to_max.y();
            auto flag = min_to_max.cross(min_to_pt).z();
            if((min_pt.x() < 0)&&(max_pt.x()>0) || (min_pt.x() > 0)&&(max_pt.x()<0)) {
                if(flag >= 0) continue;    
            }
            else{
                if(flag <= 0) continue;
            }
            float dist = std::abs(flag)/min_to_max.norm();//std::sqrt(min_to_pt.x()*min_to_pt.x() + min_to_pt.y()*min_to_pt.y());
            if(dist > max_dist && dist < min_to_max.norm()) {
                max_dist = dist;
                corner_pt = pt;
                if(corner_pt.z + h_lidar < 0.5) {
                    max_dist = std::numeric_limits<float>::lowest();
                }
            }
        }
        if(min_to_max.norm()/max_dist > 15 || min_to_max.norm() > 8) continue; 
        
        //step3.find the infer_pt point of the minBox
        if(max_dist < 0.1){
            // min_pt.x() += 0.1;
            // max_pt.x() += 0.1;
            // corner_pt.x = min_pt.x() - 0.1;
            // corner_pt.y = min_pt.y() - 0.1;
            // infer_pt.x = max_pt.x() + 0.1;
            // infer_pt.y = max_pt.y() - 0.1;
            corner_pt.x = (min_pt.x() + max_pt.x()) /2;
            corner_pt.y = (min_pt.y() + max_pt.y()) /2;
        }
        // else{
            auto diff_max_x = max_pt.x() - corner_pt.x;
            auto diff_max_y = max_pt.y() - corner_pt.y;
            auto diff_min_x = min_pt.x() - corner_pt.x;
            auto diff_min_y = min_pt.y() - corner_pt.y;
            infer_pt.x = corner_pt.x + diff_max_x + diff_min_x;
            infer_pt.y = corner_pt.y + diff_max_y + diff_min_y;
        // }
        //step4.fill the 4 points into box
        std::vector<Eigen::Vector3f> box4pt(4, Eigen::Vector3f::Zero());
        box4pt[0] = min_pt;
        box4pt[1] = corner_pt.getVector3fMap();
        box4pt[2] = max_pt;
        box4pt[3] = infer_pt.getVector3fMap();
        
        bool isPromising = FilterNormalObject(box4pt, max_z, pts_size);
        if(!isPromising) {
          continue;
        }

        PointCloud::Ptr box8pts(new PointCloud);
        PointT box_pt;
        bool temp = true;
        for(int h1 = 0; h1 < 2; h1++){
            for(int p = 0; p < 4; p++){
                if(box4pt[p].norm() < 0.8) temp = false;
                box_pt.x = box4pt[p].x();
                box_pt.y = box4pt[p].y();
                if(h1 == 0) box_pt.z = -h_lidar+0.2;
                else box_pt.z = max_z;
                box8pts->push_back(box_pt);
            }
        }
        if(!temp) continue;
        if(!box8pts->empty()) {
            obstacle->push_back(box8pts);
        }
    }
}

void Cluster::BorderMaxMinXY(const PointCloud::Ptr& cloud, Eigen::Vector3f* min_point, Eigen::Vector3f* max_point){
    //min_point max_point为某一簇点云的矩形范围框，并非边缘最小点和最大点
    Eigen::Vector3f& min_pt = *min_point;
    Eigen::Vector3f& max_pt = *max_point;
    min_pt[0] = min_pt[1] = min_pt[2] = FLT_MAX;
    max_pt[0] = max_pt[1] = max_pt[2] = -FLT_MAX;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
      max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
      min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
      max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
      min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
      max_pt[2] = std::max(max_pt[2], cloud->points[i].z);
    }
}
bool Cluster::FilterNormalObject(std::vector<Eigen::Vector3f>& box, float h, int num){
    Eigen::Vector3f min_to_max    = box[2] - box[0];
    Eigen::Vector3f min_to_corner = box[1] - box[0];
    auto area = std::abs(min_to_max.cross(min_to_corner).z());
    auto width = area/min_to_max.norm();
    auto vol  = area * h;
    auto ratio = min_to_max.norm()/width;
    if(ratio < 5 || area < 15 || vol*8 < num) return true;
    else return false;
}
void Cluster::BuildBoxs(std::vector<PointCloud::Ptr>& boxs_points, visualization_msgs::Marker& line_list){
  line_list.header.stamp = ros::Time::now();
  line_list.ns =  "boxes";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  //LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.02;
  // Points are green
  line_list.color.g = 1.0f;
  line_list.color.a = 1.0;

  int id = 0;
  std::string ids;
  for(int box_i = 0; box_i < boxs_points.size(); box_i++){
    for(int i = 0; i < 4; i++){
    //   assert((i+1)%4 < boxs_points[box_i]->size());
    //   assert((i+4) < boxs_points[box_i]->size());
    //   assert((i+1)%4+4 < boxs_points[box_i]->size());
      id ++; ids = std::to_string(id);
      
      geometry_msgs::Point p;
      p.x = boxs_points[box_i]->points[i].x;
      p.y = boxs_points[box_i]->points[i].y;

    //   if(sqrt(p.x*p.x + p.y*p.y) < 0.5 ) continue;

      p.z = boxs_points[box_i]->points[i].z;
      line_list.points.push_back(p);
      p.x = boxs_points[box_i]->points[(i+1)%4].x;
      p.y = boxs_points[box_i]->points[(i+1)%4].y;
      p.z = boxs_points[box_i]->points[(i+1)%4].z;
      line_list.points.push_back(p);

      p.x = boxs_points[box_i]->points[i].x;
      p.y = boxs_points[box_i]->points[i].y;
      p.z = boxs_points[box_i]->points[i].z;
      line_list.points.push_back(p);
      p.x = boxs_points[box_i]->points[i+4].x;
      p.y = boxs_points[box_i]->points[i+4].y;
      p.z = boxs_points[box_i]->points[i+4].z;
      line_list.points.push_back(p);

      p.x = boxs_points[box_i]->points[i+4].x;
      p.y = boxs_points[box_i]->points[i+4].y;
      p.z = boxs_points[box_i]->points[i+4].z;
      line_list.points.push_back(p);
      p.x = boxs_points[box_i]->points[(i+1)%4+4].x;
      p.y = boxs_points[box_i]->points[(i+1)%4+4].y;
      p.z = boxs_points[box_i]->points[(i+1)%4+4].z;
      line_list.points.push_back(p);
    }
  }
}
}//namespace fusion_nodelet