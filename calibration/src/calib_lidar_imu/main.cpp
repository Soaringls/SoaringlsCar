#include <iostream>
#include<vector>
#include<math.h>


#include <pcl-1.7/pcl/ModelCoefficients.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/features/normal_3d.h>
#include <pcl-1.7/pcl/filters/passthrough.h>
#include <pcl-1.7/pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/sample_consensus/method_types.h>
#include <pcl-1.7/pcl/sample_consensus/model_types.h>
#include <pcl-1.7/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.7/pcl/segmentation/extract_clusters.h>
#include <pcl-1.7/pcl/registration/icp.h>
#include<pcl-1.7/pcl/common/transforms.h>
#include<pcl-1.7/pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace std;

typedef pcl::PointXYZRGB PointT;
#define PI (3.1415926535897932346f)
/*对激光雷达和imu的外参进行标定
 * 输入为激光雷达点云，标记点坐标，imu的位置和姿态
 * 输出为激光雷达和imu的外参lidar->imu
 *
 *
 * */
class calib_imu_lidar
{
public:
    //输入文件路径，输出点云
    //succes 1,failure 0.
    int read_csv(string str,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point);

    //输入文件路径，输出点云
    //succes 0，failure 1；
    int read_txt(string str1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    //输入一帧点云.提取地面法向量,
    //succes 1,failure 0.
    int filter_floor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point,pcl::ModelCoefficients::Ptr& Normal_floor);

    //点云坐标系垂直矫正
    void turn_vetical (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point,pcl::ModelCoefficients::Ptr& Normal_floor);

    //绕任意轴旋转的矩阵 point（轴上的点）vector（单位化的轴向量）,t（旋转角度）
    Eigen::Matrix4d rot_mat(const Eigen::Vector3d& point, const Eigen::Vector3d& vector, const double t);
    //计算雷达的高度——点到平面距离
    double calpointtoplane(Eigen::Vector4d& centroid, Eigen::Vector4d& plane);
    //主程序
    int calib_process(string str,Eigen::Matrix4d& T_total,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& lidar_check_1);

    //输入一帧点云，计算两个标记桶的平面坐标
    int cal_L_2xy(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point,double radius,std::vector<Eigen::Vector2d>& cylinder_center_1);

    //计算IMu坐标系下的两个桶的位置
    std::vector<Eigen::Vector2d> cal_I_2xy();

    //计算lidar和imu的位姿
    int cal_L_I(std::vector<Eigen::Vector2d>& points_I,std::vector<Eigen::Vector2d>& points_L,Eigen::Matrix4d& T);
    //显示点云
    void Show_pcd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& lidar_points);

    //计算M_XY的另外一种方法：
    int cal_M_xy(Eigen::Matrix4d& m_xy,Eigen::Vector4d& vertical_floor);

private:
    //读取csv文件的子程序；
    string Trim(string& str);

    //线到面的交点 d_line线的法向量，Q线上一点，n_plane平面法向量，
    Eigen::Vector3d linetoplane(Eigen::Vector3d& d_line,Eigen::Vector3d& Q,Eigen::Vector3d& n_plane,double d);

    Eigen::MatrixXd pinv(Eigen::MatrixXd  A);

public:
    double height;//激光雷达距离地面的检测高度
    double imu_z;//IMU距离地面的安装高度 274mm
    Eigen::Matrix4d M_XY;

//    bool show =true;//false;
    bool show =false;

};

int calib_imu_lidar::cal_M_xy(Eigen::Matrix4d& m_xy,Eigen::Vector4d& vertical_floor)
{
    double theta_x,theta_y;

    theta_y=atan(vertical_floor(0)/vertical_floor(2));//x/z
    double M;
    M=vertical_floor(0)*sin(theta_y)+vertical_floor(2)*cos(theta_y);
    theta_x=atan(-vertical_floor(1)/M);//-y/m,m=x*sin(theta_y)+z*cos(theta_y)

    Eigen::Matrix4d mx,my;
    mx<<1,0,0,0,
        0,cos(theta_x),sin(theta_x),0,
        0,-sin(theta_x),cos(theta_x),0,
        0,0,0,1;
    my<<cos(theta_y),0,-sin(theta_y),0,
            0,1,0,0,
            sin(theta_y),0,cos(theta_y),0,
            0,0,0,1;
    m_xy=mx*my;
    return 0;

}

void calib_imu_lidar::Show_pcd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& lidar_points)
{
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.addPointCloud(lidar_points);
    viewer.addCoordinateSystem();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
int calib_imu_lidar::read_txt(string str1,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){

    ifstream file_name;
    file_name.open(str1, ios::in); //ios::in 表示以只读的方式读入文件
    // 点云文件打开失败
    if (!file_name.is_open())
    {
        cout << "open 3D datas error!" << endl;
    }
    //求点云总数目rows
     int rows = -1;
     char str[200];
     while (!file_name.eof())
     { //end of file“文件结束"，用来判断是否到达文件结尾
        file_name.getline(str, sizeof(str));
        rows++;
     }
     file_name.clear();
     file_name.seekg(0, ios::beg);//把文件的读指针从文件开头向后移0个字节(即指针指向开头的位置)
     //pcl::PointCloud<pcl::PointXYZ>cloud;
     cloud->width = rows;
     cloud->height = 1;
     cloud->is_dense = false; //判断点云中的点是否包含 Inf/NaN这种值（包含为false）
     cloud->points.resize(cloud->width*cloud->height);
//     txt数据写入点云文件
     for (int i = 0; i < rows; i++)
     {
         double num[5];
         //file_name.setf(ios::fixed, ios::floatfield);        //     设置小数精度
         //file_name.precision(7);
         file_name >> num[0]; //按txt行输入
         file_name >> num[1];
         file_name >> num[2];

         cloud->points[i].x = num[0];
         cloud->points[i].y = num[1];
         cloud->points[i].z = num[2];
         cloud->points[i].r = 255;
         cloud->points[i].g = 255;
         cloud->points[i].b = 255;

     }

    Show_pcd(cloud);
     cout<<cloud->points.size()<<endl;
     if(cloud->points.size()<10)
         return 1;
     else
         return 0;

}
int calib_imu_lidar::calib_process(string str,Eigen::Matrix4d& T_total,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& lidar_check_1) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr Normal_floor(new pcl::ModelCoefficients);

    int flag=1;
    if(flag==read_csv(str,lidar_points))
    {
        cout<<"fail to read the point csv files^^^^^^^^"<<endl;
    }
    
    if(flag==filter_floor(lidar_points,Normal_floor))
    {
        cout<<"can't get a good floor normal"<<endl;
    }
//************************check the result***************************
    std::cerr << "Model coefficients: " << Normal_floor->values[0] << " "
              << Normal_floor->values[1] << " "
              << Normal_floor->values[2] << " "
              << Normal_floor->values[3] << std::endl;

    cout<<"height="<<height<<endl;

    turn_vetical(lidar_points,Normal_floor);


    std::vector<Eigen::Vector2d> center_lidar_2xy,center_imu_2xy;

    if(flag==cal_L_2xy(lidar_points,7.0,center_lidar_2xy))
    {
        cout<<"can't get good lidar cylinders"<<endl;
        return 0;
    }

    center_imu_2xy=cal_I_2xy();

    Eigen::Matrix4d T;
    if(flag==cal_L_I(center_imu_2xy,center_lidar_2xy,T))
    {
        cout<<"failure int detect the cylinder*****";
    }

//    cout<<"center_imu_2xy:"<<center_imu_2xy[0]<<endl<<center_imu_2xy[1]<<endl;
//    cout<<"center_lidar_2xy:"<<center_lidar_2xy[0]<<endl<<center_lidar_2xy[1]<<endl;

    //Eigen::Matrix4d total_t;
    T_total=T*M_XY;

    cout<<T_total<<endl;

    pcl::transformPointCloud(*lidar_points,*lidar_check_1, T_total);

    if(show)
    {
        pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
        viewer.addPointCloud(lidar_check_1);
        viewer.addCoordinateSystem();
        while (!viewer.wasStopped())
        {
            viewer.spinOnce();
        }
    }

}

Eigen::Vector3d calib_imu_lidar::linetoplane(Eigen::Vector3d& d_line,Eigen::Vector3d& Q,Eigen::Vector3d& n_plane,double d)
{
    Eigen::Vector3d p(0,0,0);
    double delta=d_line.dot(n_plane);
    double dist=n_plane.dot(Q)+d;
    if(delta !=0)
    {
        p=Q-(dist-delta)*d_line;
    }
    return p;
}


double calib_imu_lidar::calpointtoplane(Eigen::Vector4d& centroid, Eigen::Vector4d& plane)
{
	double sample, sa, sb;
	sa = centroid.dot(plane);
	Eigen::Vector3f plane1;
	plane1 << plane(0, 0), plane(1, 0), plane(2, 0);
	sb = plane1.norm();
	sample = std::abs(sa / sb);

	return sample;
}

string calib_imu_lidar::Trim(string& str)
{
    //str.find_first_not_of(" \t\r\n"),
    str.erase(0, str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
}

int calib_imu_lidar::read_csv(string str,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point)
{
    double x, y, z;
    ifstream fin(str, ios::in);
    string line;
    while (getline(fin, line))
    {

        istringstream sin(line);
        vector<string> fields;
        string field;
        while (getline(sin, field, ','))
        {
            fields.push_back(field); //
        }
        string sx = Trim(fields[0]);
        string sy = Trim(fields[1]);
        string sz = Trim(fields[2]);

        istringstream streamx, streamy, streamz;
        streamx.str(sx), streamy.str(sy), streamz.str(sz);
        streamx >> x, streamy >> y, streamz >> z;

        pcl::PointXYZRGB point_xyz;
        point_xyz.x = x;
        point_xyz.y = y;
        point_xyz.z = z;
        point_xyz.r = 0;
        point_xyz.g = 255;
        point_xyz.b = 0;

        point->push_back(point_xyz);
    }
    fin.close();

    if (point->points.size()<10)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int  calib_imu_lidar::filter_floor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point,pcl::ModelCoefficients::Ptr& Normal_floor)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
    //对点云进行过虑 z<-0.5,-5<y<5
    pcl::PassThrough<pcl::PointXYZRGB>pass;
    pass.setInputCloud(point);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-10,1);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2,5);
    pass.filter(*cloud_filter);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud_filter);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    //    pcl::visualization::CloudViewer viewer1 ("planer viewer");
//    viewer1.showCloud(point_filter_floor);
//    while (!viewer1.wasStopped ())
//    {
//    }

//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//              << coefficients->values[1] << " "
//              << coefficients->values[2] << " "
//              << coefficients->values[3] << std::endl;
    //check the coefficients ⊥ Z 差多少度
    double error_z=0; //cos theta

    if(coefficients->values[2]<0)
    {
        Normal_floor->values[0]=-1*coefficients->values[0];
        Normal_floor->values[1]=-1*coefficients->values[1];
        Normal_floor->values[2]=-1*coefficients->values[2];
        Normal_floor->values[3]=-1*coefficients->values[3];
    }else
    {
        Normal_floor=coefficients;
    }
    Eigen::Vector3d normal_vector(Normal_floor->values[0],Normal_floor->values[1],Normal_floor->values[2]);
    error_z=Normal_floor->values[2]/normal_vector.norm();
    if(error_z>0.866 && error_z<1)//小于30度
    {

        Eigen::Vector4d zero(0.0,0.0,0.0,1.0);
        Eigen::Vector4d fplane(Normal_floor->values[0],Normal_floor->values[1],Normal_floor->values[2],Normal_floor->values[3]);
        height=calpointtoplane(zero,fplane);
        return 0;
    }else
    {
        cout<<"fail to get the floor"<<endl;
        return 1;
    }
}

void calib_imu_lidar::turn_vetical (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point,pcl::ModelCoefficients::Ptr& Normal_floor)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix4d R_T;

    Eigen::Vector3d norm(Normal_floor->values[0],Normal_floor->values[1],Normal_floor->values[2]);
    Eigen::Vector4d Normal_floor_4(Normal_floor->values[0],Normal_floor->values[1],Normal_floor->values[2],Normal_floor->values[3]);
    Eigen::Matrix3d xy_mat;
    Eigen::Vector3d z_v(0.0,0.0,1.0);

    //********************************计算M_XY******************************************
    //1,计算法向量方向，并单位化
    Eigen::Vector3d xy_axis=norm.cross(z_v);
    xy_axis.normalize();
    //2.计算旋转角度，从norm到z_v
    double t=acos(norm.dot(z_v));

    Eigen::Vector3d zero(0,0,0);
    M_XY=rot_mat(zero,xy_axis,t);

//    cout<<"M_XY"<<M_XY<<endl;
//
    //check the theta_x,theta_y
    //cout<<"result in the first turn:"<<M_XY*Normal_floor_4<<endl;

    //**********************************************************************************
    //新方法
//    Eigen::Matrix4d mxy;
//    cal_M_xy(mxy,Normal_floor_4);
//    cout<<"mxy:"<<mxy<<endl;

    //**********************************************************************************


    pcl::transformPointCloud(*point,*point_tmp, M_XY);
    point->swap(*point_tmp);

}

Eigen::Matrix4d calib_imu_lidar::rot_mat(const Eigen::Vector3d& point, const Eigen::Vector3d& vector, const double t)
{
    double u = vector(0);
    double v = vector(1);
    double w = vector(2);
    double a = point(0);
    double b = point(1);
    double c = point(2);

    Eigen::Matrix4d matrix;
    matrix<<u*u + (v*v + w*w)*cos(t), u*v*(1 - cos(t)) - w*sin(t), u*w*(1 - cos(t)) + v*sin(t), (a*(v*v + w*w) - u*(b*v + c*w))*(1 - cos(t)) + (b*w - c*v)*sin(t),
            u*v*(1 - cos(t)) + w*sin(t), v*v + (u*u + w*w)*cos(t), v*w*(1 - cos(t)) - u*sin(t), (b*(u*u + w*w) - v*(a*u + c*w))*(1 - cos(t)) + (c*u - a*w)*sin(t),
            u*w*(1 - cos(t)) - v*sin(t), v*w*(1 - cos(t)) + u*sin(t), w*w + (u*u + v*v)*cos(t), (c*(u*u + v*v) - w*(a*u + b*v))*(1 - cos(t)) + (a*v - b*u)*sin(t),
            0, 0, 0, 1;
    return matrix;
}

int calib_imu_lidar::cal_L_2xy(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point,double radius,std::vector<Eigen::Vector2d>& cylinder_center_1)
{
    std::vector<Eigen::Vector2d> cylinder_center;
//    1.选定范围radius 默认7m
//    2.删除地面，聚类选点，进行圆心拟合。
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_radius(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0;i<point->points.size();++i)
    {
        double xy_radius=point->points[i].x*point->points[i].x+point->points[i].y*point->points[i].y;
        if(xy_radius<radius*radius &&xy_radius>4 && point->points[i].y>-2.5)
        {
            point_radius->points.push_back(point->points[i]);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_filter_floor(new pcl::PointCloud<pcl::PointXYZRGB>);
    //去地面
    //存放局内点索引
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // 选择模型系数是否需要优化
    seg.setOptimizeCoefficients (true);
    // 设置模型类型
    seg.setModelType (pcl::SACMODEL_PLANE);
    // 设置模型估计方法
    seg.setMethodType (pcl::SAC_RANSAC);
    // 设置最大迭代次数
    seg.setMaxIterations(10000);
    // 距离阈值（离散点到模型表面距离）
    seg.setDistanceThreshold (0.2);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // 输入点云
    seg.setInputCloud (point_radius);
    // 计算模型参数和得到符合此模型的局内点索引
    seg.segment (*inliers, *coefficients);
    // 输出平面模型的四个参数
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                       << coefficients->values[2] << " "
                                       << coefficients->values[3] << std::endl;

    extract.setInputCloud(point_radius);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*point_filter_floor);
    //cout<<"聚类点云样本个数："<<point_filter_floor->points.size()<<endl;

     if(show)
     {
         pcl::visualization::CloudViewer viewer1 ("planer filter viewer");
        viewer1.showCloud(point_filter_floor);
        while (!viewer1.wasStopped ())
        {
        }
     }



    //聚类的点云求圆柱

    //聚类
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud (point_filter_floor);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.3); //设置近邻搜索的搜索半径为2cm
    ec.setMinClusterSize (80);//设置一个聚类需要的最少点数目为100
    ec.setMaxClusterSize (2000); //设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree);//设置点云的搜索机制
    ec.setInputCloud (point_filter_floor);
    ec.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

    //cout<<"聚类个数："<<cluster_indices.size()<<endl;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(point_filter_floor->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);

        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_cluster);
        ne.setKSearch(30);
        ne.compute(*cloud_normals);

        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg1;
        seg1.setOptimizeCoefficients(true);
        seg1.setModelType(pcl::SACMODEL_CYLINDER);
        seg1.setMethodType(pcl::SAC_RANSAC);
        seg1.setNormalDistanceWeight(0.1);
        seg1.setMaxIterations(10000);
        seg1.setDistanceThreshold(0.1);
        seg1.setRadiusLimits(0.12, 0.25);

        seg1.setInputCloud(cloud_cluster);
        seg1.setInputNormals(cloud_normals);

        seg1.segment(*inliers_cylinder, *coefficients_cylinder);
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

//        std::cerr << "inliers_cylinder : "<< inliers_cylinder->indices.size()<<std::endl;

        pcl::ExtractIndices<pcl::PointXYZRGB> extract1;
        extract1.setInputCloud(cloud_cluster);
        extract1.setIndices(inliers_cylinder);
        extract1.setNegative(false);
        pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
        extract1.filter(*cloud_cylinder);

        if (cloud_cylinder->points.empty()) {
            std::cerr << "Can't find the cylindrical component." << std::endl;

        } else {
            //show the clinder

//            cout << "圆柱点:" << cloud_cylinder->size() << endl;
//            cout << "聚类点:" << cloud_cluster->size() << endl;

            double key_num=double(cloud_cylinder->size())/double(cloud_cluster->size());//验证是否圆柱

            if(key_num >0.75)
            {

                if(show)
                {
                    pcl::visualization::CloudViewer viewer2("cylinder viewer");
                    viewer2.showCloud(cloud_cylinder);
                    while (!viewer2.wasStopped()) {
                    }
                }

                //计算圆柱与地面的交点
                Eigen::Vector3d Q,d_line,n;
                double d;
                Q<<coefficients_cylinder->values[0], coefficients_cylinder->values[1],coefficients_cylinder->values[2];
                d_line<<coefficients_cylinder->values[3], coefficients_cylinder->values[4],coefficients_cylinder->values[5];
                cout<<"圆柱半径："<<coefficients_cylinder->values[6]<<endl;

                n<<0,0,1;
                d=-height;

                Eigen::Vector3d p;
                p=linetoplane(d_line,Q,n,d);

                Eigen::Vector2d center(p(0),p(1));

                cylinder_center.push_back(center);
                //cout << "center:" << center << endl;
            }
        }
    }

    if(cylinder_center.size()<2)
    {
        cout<<"fail to get 2 cylinder in lidar*******"<<endl;
        return 1;
    }

    if(cylinder_center[0](0)<cylinder_center[1](0))
        cylinder_center_1=cylinder_center;
    else
    {
        cout<<"转换了坐标位置"<<endl;
        cylinder_center_1.push_back(cylinder_center[1]);
        cylinder_center_1.push_back(cylinder_center[0]);
    }

    return 0;

}

std::vector<Eigen::Vector2d> calib_imu_lidar::cal_I_2xy()
{
    std::vector<Eigen::Vector2d> center_I_2xy;

//    Eigen::Vector3d P1(456373.503771,4425313.88329,1),P2(456363.183255,4425313.99151,1);
    Eigen::Vector3d P1(777708.122953,3501611.42615,1),P2(777700.622334,3501607.09658,1);

    Eigen::Vector3d p_imu;
//    p_imu<<456368.026177,4425318.08686,1;
    p_imu<<777705.594667,3501616.10836,1;

    double imu_height=20.6481666565;

//    Eigen::Quaterniond q(-0.013423230977,0.0020370811945,0.00973499178654,-0.999860438814);
    Eigen::Quaterniond q(0.00257280545403, -0.00143453710165,-0.015833061192,-0.999870310065 );

    Eigen::Matrix3d Rx = q.toRotationMatrix();
    //cout << Rx << endl << endl;

    Eigen::Vector3d ea1 = Rx.eulerAngles(2,1,0);
    //cout << ea1/PI*180 << endl << endl;

    double theta=ea1(0);

    Eigen::Matrix3d T_I_U,T_U_I;
    T_I_U<<cos(theta),sin(theta),p_imu(0),
        -sin(theta),cos(theta),p_imu(1),
        0,0,1;

    T_U_I=T_I_U.inverse();

//    Eigen::Vector3d aa(0,0,1);
//    cout<<T_I_U*aa<<endl<<endl;
//
//    cout<<T_U_I*p_imu<<endl;

    Eigen::Vector3d P1_POSITION,P2_POSITION;
    P1_POSITION=T_U_I*P1;
    P2_POSITION=T_U_I*P2;

    Eigen::Vector2d p1(P1_POSITION(0),P1_POSITION(1)),p2(P2_POSITION(0),P2_POSITION(1));

    center_I_2xy.push_back(p1);
    center_I_2xy.push_back(p2);

    //cout<<center_I_2xy[0]<<endl<<center_I_2xy[1]<<endl;
    return center_I_2xy;
    //对绕z的角度进行处理，即ea1(0).


}

int calib_imu_lidar::cal_L_I(std::vector<Eigen::Vector2d>& points_I,std::vector<Eigen::Vector2d>& points_L,Eigen::Matrix4d& T)
{
//    Eigen::MatrixXf m(3,3);
//    m << 1,2,3, 4,5,6, 7,8,9;
//    cout << "Here is the matrix m:" << endl << m << endl;
//    cout << "2nd Row: " << m.row(1) << endl;
//    m.col(2) += 3 * m.col(0);
//    cout << "After adding 3 times the first column into the third column, the matrix m is:\n";
//    cout << m << endl;
        cout<<"lidar:"<<points_L[0]<<" "<<points_L[1]<<endl;
        cout<<"IMU:"<<points_I[0]<<" "<<points_I[1]<<endl;


        Eigen::Vector2d aa,bb;
        aa=points_I[0]-points_I[1];
        bb=points_L[0]-points_L[1];

        cout<<"aa:"<<aa.norm()<<endl;
        cout<<"bb:"<<bb.norm()<<endl;

        if(abs(aa.norm()-bb.norm())>0.5)
        {
            cout<<"fail to detect the  cylinder……"<<endl;
            return 1;
        }
    //**************************************************************************************************************************
    //当检测的雷达点和imu输出的点，存在误差时，下面的程序选择的方式是：假设sin值求出的角度是准确的，保持旋转角度方程模型，而非完全的解析解替换M_z。*********
    //*************************************************************       *****************************************************
        //求从L到I的角度信息z_axis;
//        double z_axis,tmp;
////        tmp=(aa(0)*bb(0)+aa(1)*bb(1))/(bb.norm()*bb.norm());
////        z_axis=acos(tmp);
//
//        tmp=(aa(0)*bb(1)+aa(1)*bb(0))/(bb.norm()*bb.norm());
//        z_axis=asin(tmp);
//
//        cout<<"旋转角度为："<<z_axis<<endl;
//
//        Eigen::Matrix2d M_z;
//        M_z<<cos(z_axis),sin(z_axis),
//                -sin(z_axis),cos(z_axis);
//        cout<<"M_z："<<M_z<<endl;
    //方案2：解析解求解M_z，不考虑sin2+cos2=1；
        double z_axis,tmp_s,tmp_c;
        tmp_c=(aa(0)*bb(0)+aa(1)*bb(1))/(bb.norm()*bb.norm());
        tmp_s=(aa(0)*bb(1)-aa(1)*bb(0))/(bb.norm()*bb.norm());
        Eigen::Matrix2d M_z;
        M_z<<tmp_c,tmp_s,
                -tmp_s,tmp_c;
//        cout<<"M_z："<<M_z<<endl;
        cout<<"旋转角度为："<< asin(tmp_s)/PI*180 << endl << endl;

        double dx,dy,dz;
        Eigen::Vector2d a_dxdy,b_dxdy;
        a_dxdy=points_I[0]-M_z*points_L[0];
        b_dxdy=points_I[1]-M_z*points_L[1];

//        cout<<"a_dxdy:"<<a_dxdy<<endl;
//        cout<<"b_dxdy:"<<b_dxdy<<endl;

        dx=(a_dxdy(0)+b_dxdy(0))/2;
        dy=(a_dxdy(1)+b_dxdy(1))/2;

        imu_z=0.274;
        dz=height-imu_z;
        //dz=0;

        //Eigen::Matrix4d T;
        T<<M_z(0,0),M_z(0,1),0,dx,
            M_z(1,0),M_z(1,1),0,dy,
            0,0,1,dz,
            0,0,0,1;
        //check
        Eigen::Vector4d a_l,a_i;
        a_l<<points_L[0](0),points_L[0](1),0,1;
        a_i<<points_I[0](0),points_I[0](1),0,1;

//        cout<<a_i<<endl;
//        cout<<T*a_l<<endl;


    return 0;
    
}

Eigen::MatrixXd calib_imu_lidar::pinv(Eigen::MatrixXd  A)//求矩阵的广义逆
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
    double  pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*

    return X;

}

int main() {


    calib_imu_lidar a;

    string left,right,top ,bottom;
    left="result-208/p2/L2-208.csv";
    right="result-208/p2/R2-208.csv";
    top="result-208/p2/T2-208.csv";
    bottom="result-208/p2/B2-208.txt";
//    left="result-208/p1/L1-208.csv";
//    right="result-208/p1/R1-208.csv";
//    top="result-208/p1/T1-208.csv";

//    Eigen::Matrix4d T_left,T_right,T_top,T_bottom;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lidar_check_left(new pcl::PointCloud<pcl::PointXYZRGB>),
//            lidar_check_right(new pcl::PointCloud<pcl::PointXYZRGB>),
//            lidar_check_top(new pcl::PointCloud<pcl::PointXYZRGB>),
//            lidar_check_bottom(new pcl::PointCloud<pcl::PointXYZRGB>),
//            lidar_check(new pcl::PointCloud<pcl::PointXYZRGB>);
//    a.calib_process(left,T_left,lidar_check_left);
//    a.calib_process(right,T_right,lidar_check_right);
//    a.calib_process(top,T_top,lidar_check_top);
////    a.calib_process(bottom,T_bottom,lidar_check_bottom);
//
//    *lidar_check+=*lidar_check_left;
//    *lidar_check+=*lidar_check_right;
//    *lidar_check+=*lidar_check_top;
////    *lidar_check+=*lidar_check_bottom;

    std::vector<Eigen::Vector2d> check_1;
    check_1=a.cal_I_2xy();
    cout<<check_1[0]<<endl<<check_1[1]<<endl;

//    a.Show_pcd(lidar_check);

    return 0;
}