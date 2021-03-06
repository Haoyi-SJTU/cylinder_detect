#include <iostream>
#include <RVC/RVC.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <stdio.h>
#include <ctime>
#include <cstdio>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>                 // 带通滤波器
#include <pcl/features/normal_3d.h>                  // 法向量估计
#include <pcl/segmentation/sac_segmentation.h>       // 模型分割
#include <pcl/filters/extract_indices.h>             // 索引提取
#include <pcl/filters/statistical_outlier_removal.h> // 点云离群值移除
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>            //体素
#include <pcl/segmentation/extract_clusters.h> // 聚类算法
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/io.h>
#include <pcl/registration/correspondence_estimation.h>

#include <cstdlib>
#include <cmath>

#include <boost/thread/thread.hpp>
#include <run_rvc/run_rvc.h>
#include <dirent.h> //DIR *dir
// ROS 发布点云消息
// #include <ros/ros.h>
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/transforms.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/io/pcd_io.h>
// #include <tf/transform_datatypes.h>

// 使用ROS请求服务
#include <ros/ros.h>
#include <couple/result.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

using namespace std;

clock_t Start, End;
typedef pcl::PointXYZ PointT;
// std::fstream _file;

float eye[4][4] =
    // {{-0.0330089, 0.619485, -0.784314, 0.793654},
    //  {0.0297455, 0.785003, 0.618777, 0.0247557},
    //  {0.999012, -0.00290472, -0.044339, 0.596428},
    //  {0, 0, 0, 1}};
      // {{0.105301,   0.679102,  -0.726452,      0.6621},
      //  {-0.0502465,   0.733214,   0.678139,    0.00778075},
      //  {0.99317, -0.0349071,   0.111331,   -0.0115626},
      //  {0, 0, 0, 1}};
 {{0.0989764 ,  0.686197,  -0.720651,    0.689631+0.05},
{-0.0741222,   0.727279,   0.682328,   -0.00380715},
  {0.992325, -0.0141181,   0.122846,   -0.0317281-0.02},//-0.0317281
         {0,          0,          0,          1}};

// 数组乘法计算
// 输入：eye手眼标定矩阵，point待变换的坐标
// 输出：result变换结果，以指针传递
int multi(float eye[4][4], float point[4][1], float result[4][1])
{
    int b = 0;
    for (int a = 0; a < 4; ++a)
    {
        result[a][0] = 0;
        for (int b = 0; b < 4; ++b)
        {
            result[a][0] = result[a][0] + eye[a][b] * point[b][0];
        }
    }
    return 1;
}

// 发布自定义ROS消息
int talker(ros::ServiceClient client, bool type, float distance, float x, float y, float z, float a, float b, float c, float d, float e, float f)
{
    float m = a + d;
    float n = b + e;
    float p = c + f;
    float ab = sqrt((a - m) * (a - m) + (b - n) * (b - n) + (c - p) * (c - p));
    float ap = sqrt((a - x) * (a - x) + (b - y) * (b - y) + (c - z) * (c - z));
    float bp = sqrt((m - x) * (m - x) + (n - y) * (n - y) + (p - z) * (p - z));
    float cosa = (ap * ap + ab * ab - bp * bp) / (2 * ab * ap);
    float sina = sqrt(1 - cosa * cosa);
    // cout<<"distance"<<ap*sina<<endl;
    float point[4][1] = {{x}, {y}, {z}, {1}};
    float result2[4][1] = {0};
    multi(eye, point, result2);

    float point0[4][1] = {{a}, {b}, {c}, {1}};
    float result02[4][1] = {0};
    multi(eye, point0, result02);

    float point1[4][1] = {{d}, {e}, {f}, {0}};
    float result12[4][1] = {0};
    multi(eye, point1, result12);

    couple::result srv;
    srv.request.type = type;
    srv.request.distance = distance;
    srv.request.x = result2[0][0];
    srv.request.y = result2[1][0];
    srv.request.z = result2[2][0];
    srv.request.a = result02[0][0];
    srv.request.b = result02[1][0];
    srv.request.c = result02[2][0];
    srv.request.d = result12[0][0];
    srv.request.e = result12[1][0];
    srv.request.f = result12[2][0];
    cout << result2[0][0] << result2[1][0] << result2[2][0] << endl;
    cout << result02[0][0] << result02[1][0] << result02[2][0] << endl;
    cout << result12[0][0] << result12[1][0] << result12[2][0] << endl;
    while (!client.call(srv))
    {
        cout << "妹有发送成功" << endl;
    }
    cout << "result of planning" << (bool)srv.response.status << endl;
    return (bool)srv.response.status;
}

// 输入源点云和剪刀点云，输出源点云中去除和剪刀重叠部分的点云指针
pcl::PointCloud<pcl::PointXYZ>::Ptr cut_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cylinder)
{
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> core;
    core.setInputSource(cloud);
    core.setInputTarget(cut_cylinder);
    boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);
    core.determineReciprocalCorrespondences(*cor, 0.0051);                                //对应点之间的最大距离
    vector<int> pointIdxVec_A;                                                            // 构造重叠点云的索引
    pcl::registration::getQueryIndices(*cor, pointIdxVec_A);                              // 获取重叠部分对应于setInputSource()输入点云的索引
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_overlapA(new pcl::PointCloud<pcl::PointXYZ>); //根据索引提取点云
    pcl::ExtractIndices<pcl::PointXYZ> extrA;
    extrA.setInputCloud(cloud);                                       //设置输入点云
    extrA.setIndices(boost::make_shared<vector<int>>(pointIdxVec_A)); //设置索引
    extrA.setNegative(true);                                          // 提取对应索引之外的点
    extrA.filter(*non_overlapA);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor3;
    sor3.setInputCloud(non_overlapA);
    sor3.setMeanK(30);            //50个临近点
    sor3.setStddevMulThresh(1.0); //距离大于1倍标准方差
    sor3.filter(*non_overlapA);
    return (non_overlapA);
}

int main(int argc, char **argv)
{

    RVC::X1 x1 = open_fucking_rvc();

    // string g_sSamplePath = "./train/";

    DIR *dir;
    if ((dir = opendir("/home/aemc/catkin_ws/devel/lib/rvv/out/")) == NULL)
    {
        system("mkdir -p /home/aemc/catkin_ws/devel/lib/rvv/out/");
    }

    FILE *DATA;
    double type = 0;
    // double plane_distance = 0;
    DATA = fopen("/home/aemc/catkin_ws/devel/lib/rvv/out/result.txt", "r");
    if (DATA == NULL)
    {
        cout << "妹找到txt文件啊" << endl;
    }
    else
    {
        fscanf(DATA, "%lf", &type);
    }

    time_t nowtime;
    struct tm *p; //时间结构体指针 p
    time(&nowtime);
    p = localtime(&nowtime); //调用本地时间函数localtime将nowtime变量的日历时间转化为本地时间，存入指针p
    int hour = p->tm_hour;   // 在上午9点到下午4点之间3D曝光时间18%，中午11点到14点曝光时间12%，其它时间30%
    RVC::X1::CaptureOptions temp;
    if (hour >= 9 && hour < 16)
    {
        temp.exposure_time_3d = 18;
        if (hour >= 11 && hour <= 14)
            temp.exposure_time_3d = 12;
    }
    else
    {
        temp.exposure_time_3d = 35;
    }
    // temp.exposure_time_2d = 3;

    ros::init(argc, argv, "talker");
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<couple::result>("talker");

L1:
    Start = clock();
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (x1.Capture(temp) == true)
    {
        std::cout << "在拍了在拍了!" << std::endl;
        RVC::PointMap pm = x1.GetPointMap();
        cloud = PointMap2CloudPoint(pm);
        End = clock();
    }
    else
    {
        std::cout << "拍照失败!" << std::endl;
    }

    End = clock();
    double endtime = (double)(End - Start) / CLOCKS_PER_SEC;
    cout << "capture time:" << endtime << endl;

    if (cloud->points.empty())
    {
        PCL_ERROR("点云为空，即将重新拍照…");
        goto L1;
    }

    // 删除点云无效点
    // cloud_in->is_dense = false
    // 为了防止去除点云无效值失败，手动设置点云标记为“存在无效值”
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

    pcl::PassThrough<pcl::PointXYZ> pass0;
    pass0.setFilterFieldName("y");
    pass0.setInputCloud(cloud);
    pass0.setFilterLimits(-0.8, 0.6);
    pass0.filter(*cloud); //直接在输入点云cloud上裁剪

    // 降采样
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.005, 0.005, 0.005); //单位是什么？(0.003f, 0.003f, 0.003f)
    sor.filter(*cloud);                   //

    {
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ>("/home/aemc/catkin_ws/devel/lib/rvv/out/original.pcd", *cloud, false);
    }

    // 提取平面并删除
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg0;
    seg0.setOptimizeCoefficients(true);
    seg0.setModelType(pcl::SACMODEL_PLANE);
    seg0.setMethodType(pcl::SAC_RANSAC);
    seg0.setDistanceThreshold(0.01);
    seg0.setInputCloud(cloud);
    seg0.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("没找到平面，即将重新拍照…");
        goto L1;
    }
    //平面的系数  ax + by + cz +d =0;
    cout << "平面方程: " << coefficients->values[0] << "x+" << coefficients->values[1] << "y+"
         << coefficients->values[2] << "z+" << coefficients->values[3] << "=0" << std::endl;
    // 计算当前识别的平面相对标准平面的偏差  -0.0929844x+0.721556y+0.686084z+-3.38792=0   -0.106776x+-0.701113y+0.70501z+-1.1997=0
    float aa = (-abs(coefficients->values[0]) -0.106776) / 2;
    float bb = (abs(coefficients->values[1]) + 0.701113) / 2;
    float cc = (abs(coefficients->values[2]) + 0.70501) / 2;
    float dd = coefficients->values[3] + 1.1997;
    float distance = -dd / (sqrt(aa * aa + bb * bb + cc * cc)); //distance=-dd/(sqrt(aa*aa+bb*bb+cc*cc));
    if (abs(distance) < 0.9)                                    // 仅当当前平面平行于标准平面，且两者距离不超过0.9m时
    {
        cout << "车车平移距离：" << distance << endl;
    }
    else
    {
        PCL_ERROR("车车位置超限，即将重新拍照…");
        // goto L1;
    }

    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract00; // 创建索引提取点对象
    extract00.setInputCloud(cloud);        // 设置输入点云：待分割点云
    extract00.setIndices(inliers);         // 设置内点索引
    extract00.setNegative(true);           // true，提取平面外点
    extract00.filter(*cloud2);             // 执行滤波，并将结果点云保存到cloud中

    // 分割指定阈值内的平面
    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    double D = coefficients->values[3];
    vector<int> pointIdxVec;
    for (int i = 0; i < cloud2->points.size(); ++i)
    {
        float x0 = cloud2->points[i].x;
        float y0 = cloud2->points[i].y;
        float z0 = cloud2->points[i].z;
        float point2plane_distance = (A * x0 + B * y0 + C * z0 + D) / sqrt(A * A + B * B + C * C); //计算点到平面的距离
        if (point2plane_distance < -0.05)                                                          //距离阈值
        {
            pointIdxVec.push_back(i); // 予以保留的点
        }
    }
    pcl::copyPointCloud(*cloud2, pointIdxVec, *cloud2);

    // {pcl::PCDWriter writer;
    //  writer.write<pcl::PointXYZ> ("/home/aemc/catkin_ws/devel/lib/rvv/out/remove_plane.pcd", *cloud2, false);}

    { // 第二次平面移除
        pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg0.setOptimizeCoefficients(true);
        seg0.setModelType(pcl::SACMODEL_PLANE);
        seg0.setMethodType(pcl::SAC_RANSAC);
        seg0.setDistanceThreshold(0.01);
        seg0.setInputCloud(cloud2);
        seg0.segment(*inliers, *coefficients2);
        if (inliers->indices.size() != 0)
        {
            cout << "second plane:" << inliers->indices.size() << endl;
            pcl::ExtractIndices<PointT> extract00; // 创建索引提取点对象
            extract00.setInputCloud(cloud2);       // 设置输入点云：待分割点云
            extract00.setIndices(inliers);         // 设置内点索引
            extract00.setNegative(true);           // true，提取平面外点
            extract00.filter(*cloud2);             // 执行滤波，并将结果点云保存到cloud中
        }
    }

    // 做圆柱拟合前先清理干净噪点
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
        sor2.setInputCloud(cloud2);
        sor2.setMeanK(5);             //临近点
        sor2.setStddevMulThresh(0.5); //距离大于1倍标准方差
        sor2.filter(*cloud2);
    }

    pcl::PointCloud<pcl::PointXYZ> cloud2_copy_temp;
    cloud2_copy_temp = *cloud2; // 原始点云的深拷贝
    pcl::PointCloud<PointT>::Ptr cloud2_copy(new pcl::PointCloud<PointT>);
    *cloud2_copy = cloud2_copy_temp; //建立拷贝点云的指针

    // 通过聚类算法进一步清理噪点
    {
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
        ece.setInputCloud(cloud2);
        ece.setClusterTolerance(0.01);
        ece.setMinClusterSize(10);
        ece.setMaxClusterSize(2500);
        pcl::search::KdTree<PointT>::Ptr tree3(new pcl::search::KdTree<PointT>());
        ece.setSearchMethod(tree3);
        ece.extract(cluster_indices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr part_of_julei(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_copy(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                part_of_julei->push_back((*cloud2)[*pit]);
            part_of_julei->width = part_of_julei->size();
            part_of_julei->height = 1;
            part_of_julei->is_dense = true;
            if (part_of_julei->size() > 100)
            {
                *cloud2_copy = (*part_of_julei + *cloud2_copy);
            }
        }
        *cloud2 = *cloud2_copy;
    }

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("x");
    pass.setInputCloud(cloud2);

    pcl::PointXYZ cloud2_min; //用于存放三个轴的最小值
    pcl::PointXYZ cloud2_max; //用于存放三个轴的最大值
    pcl::getMinMax3D(*cloud2, cloud2_min, cloud2_max);

    pcl::NormalEstimation<PointT, pcl::Normal> ne; //创建法向量估计对象
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree); //设置搜索方式
    ne.setKSearch(50);        //设置K近邻搜索点的个数

    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;                      // 创建圆柱体分割对象
    seg.setOptimizeCoefficients(true);                                             // 设置对估计的模型系数需要进行优化
    seg.setModelType(pcl::SACMODEL_CYLINDER);                                      // 设置分割模型为圆柱体模型
    seg.setMethodType(pcl::SAC_RANSAC);                                            // 设置采用RANSAC算法进行参数估计
    seg.setNormalDistanceWeight(0.016);                                            // 设置表面法线权重系数
    seg.setAxis({1, 0, 0});                                                        //x轴方向
    seg.setEpsAngle(0.5);                                                          //偏离角度（弧度制）
    seg.setMaxIterations(3000);                                                    // 设置迭代的最大次数
    seg.setDistanceThreshold(0.008);                                               // 设置内点到模型距离的最大值
    seg.setRadiusLimits(0.0095, 0.016);                                            // 设置圆柱模型半径的范围
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);                // 保存分割结果
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients); // 保存圆柱体模型系数
    float flag = 0;
    int counter = 0;
    float Xmin = cloud2_min.x + 0.02;
    float Xmax = cloud2_max.x - 0.02;
    float Xdelta = 0.05;
    //条件：圆柱轴线近似平行于X轴。阈值：0.5
    // 设置counter是为了避免陷入死循环，循环三次后即使没有正确结果也强制退出
    while (flag < 0.1 && counter <= 1)
    {
        // {pcl::PCDWriter writer;
        //  writer.write<pcl::PointXYZ> ("/home/aemc/catkin_ws/devel/lib/rvv/out/filter0.pcd", *cloud2, false);}
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterLimits(Xmin, Xmax);
        pass.filter(*cloud2);     //直接在输入点云cloud上裁剪
        ne.setInputCloud(cloud2); //设置输入点云
        ne.compute(*cloud_normals);

        seg.setInputCloud(cloud2); //待分割点云
        seg.setInputNormals(cloud_normals);
        seg.segment(*inliers_cylinder, *coefficients_cylinder);
        flag = coefficients_cylinder->values[6]; //用flag记录半径
        cout << counter << "正在搜索向量(" << coefficients_cylinder->values[3] << "," << coefficients_cylinder->values[4] << "," << coefficients_cylinder->values[5] << ")" << endl;
        counter++;
        Xmin = Xmin + Xdelta;
        Xmax = Xmax - Xdelta;
    }
    // 存放轴线函数的点斜式
    float a = float(coefficients_cylinder->values[0]);
    float b = float(coefficients_cylinder->values[1]);
    float c = float(coefficients_cylinder->values[2]);
    float d = float(coefficients_cylinder->values[3]);
    float e = float(coefficients_cylinder->values[4]);
    float f = float(coefficients_cylinder->values[5]);

    cout << "把手方向向量(" << d << "," << e << "," << f << ")" << endl;
    cout << "把手半径" << coefficients_cylinder->values[6] << endl;

    // 抠出来把手圆柱点云
    pcl::ExtractIndices<PointT> extract;  // 创建索引提取点对象
    extract.setInputCloud(cloud2);        // 设置输入点云：待分割点云
    extract.setIndices(inliers_cylinder); // 设置内点索引
    extract.setNegative(false);           // 默认false，提取圆柱体内点；true，提取圆柱体外点
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    // pcl::PointCloud<PointT>::Ptr cloud_cylinder1(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder); // 执行滤波，并将结果点云保存到cloud_cylinder中

    cout << "圆柱之点数" << cloud_cylinder->size() << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    if (!cloud_cylinder->points.empty()) //如果把手点云非空
    {
        // 点云离群值移除
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
        sor2.setInputCloud(cloud_cylinder);
        sor2.setMeanK(5);           //临近点
        sor2.setStddevMulThresh(1); //距离大于1倍标准方差
        sor2.filter(*cloud_cylinder);
        // cout<<"OJBK11111"<<endl;
        pcl::PointXYZ min0; //用于存放三个轴的最小值
        pcl::PointXYZ max0; //用于存放三个轴的最大值
        pcl::getMinMax3D(*cloud_cylinder, min0, max0);
        pcl::PointCloud<pcl::PointXYZ> copy0;
        copy0 = *cloud; // 原始点云的深拷贝
        pcl::PointCloud<PointT>::Ptr copy1(new pcl::PointCloud<PointT>);
        *copy1 = copy0; //建立拷贝点云的指针
        pcl::PassThrough<pcl::PointXYZ> pass0;
        pass0.setFilterFieldName("x");
        pass0.setInputCloud(copy1);
        pass0.setFilterLimits(min0.x - 0.05, max0.x + 0.05);
        pass0.filter(*copy1); //直接在输入点云上裁剪

        pass0.setFilterFieldName("y");
        pass0.setInputCloud(copy1);
        pass0.setFilterLimits(min0.y - 0.03, max0.y + 0.03);
        pass0.filter(*copy1); //直接在输入点云cloud上裁剪
        // cout<<copy->points.size()<<endl;

        pass0.setFilterFieldName("z");
        pass0.setInputCloud(copy1);
        pass0.setFilterLimits(min0.z - 0.03, max0.z + 0.03);
        pass0.filter(*copy1); //直接在输入点云cloud上裁剪

        //点云欧式聚类
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
        ece.setInputCloud(copy1);
        ece.setClusterTolerance(0.01);
        ece.setMinClusterSize(10);
        ece.setMaxClusterSize(2500);
        pcl::search::KdTree<PointT>::Ptr tree3(new pcl::search::KdTree<PointT>());
        ece.setSearchMethod(tree3);
        ece.extract(cluster_indices);

        int max_point = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
        // 遍历各个聚类结果，找出来点数最大的，认为是纯粹的把手圆柱
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cloud_cluster->push_back((*copy1)[*pit]);
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            if (cloud_cluster->size() > max_point)
            {
                *cut_cylinder = (*cloud_cluster);
                max_point = cloud_cluster->size();
                cout << cut_cylinder->points.size() << endl;
            }
        }

        *cut_cylinder = *copy1;
        cout << "圆柱之点数" << cut_cylinder->size() << endl;
        // 存储识别到的把手
        {
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("/home/aemc/catkin_ws/devel/lib/rvv/out/cylinder.pcd", *cut_cylinder, false);
        }
        cout << "cylinder OJBK" << endl;

        // 计算圆柱轴线两个端点的坐标
        pcl::PointXYZ min; //用于存放三个轴的最小值
        pcl::PointXYZ max; //用于存放三个轴的最大值
        pcl::getMinMax3D(*cut_cylinder, min, max);
        float y_min = (min.x - a) * e / d + b;
        float z_min = (min.x - a) * f / d + c;
        cout << "呐呐呐把手下端点(" << min.x << "," << y_min << "," << z_min << ")" << endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr non_overlapA(new pcl::PointCloud<pcl::PointXYZ>);
        non_overlapA = cut_pointcloud(cloud, cut_cylinder);
        // {// 存储用于点云避障的点云文件，该文件是原始点云经降采样、去把手、离群值移除后的结果
        //     pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>core;
        //     core.setInputSource(cloud);
        //     core.setInputTarget(cut_cylinder);
        //     boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);
        //     core.determineReciprocalCorrespondences(*cor, 0.0051);//对应点之间的最大距离
        //     vector<int>pointIdxVec_A;// 构造重叠点云的索引
        //     pcl::registration::getQueryIndices(*cor, pointIdxVec_A); // 获取重叠部分对应于setInputSource()输入点云的索引
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr non_overlapA(new pcl::PointCloud<pcl::PointXYZ>); //根据索引提取点云
        //     pcl::ExtractIndices<pcl::PointXYZ> extrA;
        //     extrA.setInputCloud(cloud);//设置输入点云
        //     extrA.setIndices(boost::make_shared<vector<int>>(pointIdxVec_A));//设置索引
        //     extrA.setNegative(true);   // 提取对应索引之外的点
        //     extrA.filter(*non_overlapA);
        //     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor3;
        //     sor3.setInputCloud(non_overlapA);
        //     sor3.setMeanK(30);//50个临近点
        //     sor3.setStddevMulThresh(1.0);//距离大于1倍标准方差
        //     sor3.filter(*non_overlapA);

        // }
        {
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("/home/aemc/catkin_ws/devel/lib/rvv/out/collision.pcd", *non_overlapA, false);
        }

        // 去掉已经识别出来的圆柱
        pcl::ExtractIndices<PointT> extract2;  //创建索引提取点对象
        extract2.setInputCloud(cloud2);        //输入：待分割点云
                                               // 注意！这里用的是原始降采样的点云，没有做平面移除
        extract2.setIndices(inliers_cylinder); //设置内点索引
        extract2.setNegative(true);            //true，提取圆柱体外点
        pcl::PointCloud<PointT>::Ptr plus(new pcl::PointCloud<PointT>());
        extract2.filter(*plus);
        // 对抠掉把手的剩余点云做离群值移除
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor3;
        sor3.setInputCloud(plus);
        sor3.setMeanK(30);            //50个临近点
        sor3.setStddevMulThresh(1.0); //距离大于1倍标准方差
        sor3.filter(*plus);

        if (!plus->points.empty()) //如果把手点云非空
        {

            pcl::PassThrough<pcl::PointXYZ> pass2;
            pass2.setFilterFieldName("x");
            pass2.setInputCloud(plus); //直接在输入点云cloud上裁剪

            pcl::NormalEstimation<PointT, pcl::Normal> ne2; //创建法向量估计对象
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<PointT>::Ptr tree2(new pcl::search::KdTree<PointT>());
            ne2.setSearchMethod(tree2); //设置搜索方式
            ne2.setKSearch(50);         //设置K近邻搜索点的个数
            // ne2.setInputCloud(plus);//设置输入点云

            pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg2;                      // 创建圆柱体分割对象
            seg2.setOptimizeCoefficients(true);                                             // 设置对估计的模型系数需要进行优化
            seg2.setModelType(pcl::SACMODEL_CYLINDER);                                      // 设置分割模型为圆柱体模型
            seg2.setMethodType(pcl::SAC_RANSAC);                                            // 设置采用RANSAC算法进行参数估计
            seg2.setNormalDistanceWeight(0.016);                                            // 设置表面法线权重系数
            seg2.setMaxIterations(3000);                                                    // 设置迭代的最大次数
            seg2.setAxis({0, 1, 0});                                                        //y轴方向
            seg2.setEpsAngle(0.9);                                                          //偏离角度（弧度制）
            seg2.setDistanceThreshold(0.008);                                               // 设置内点到模型距离的最大值
            seg2.setRadiusLimits(0.01, 0.016);                                              // 设置圆柱模型半径的范围
            pcl::PointIndices::Ptr inliers_cylinder2(new pcl::PointIndices);                // 保存分割结果
            pcl::ModelCoefficients::Ptr coefficients_cylinder2(new pcl::ModelCoefficients); // 保存圆柱体模型系数
            flag = 1;
            int point_num = 0;
            counter = 0;
            // float distance=1;//两圆柱中心距离之平方
            float aa = 0;
            float bb = 0;
            float cc = 0;
            // Xdelta = 0.05;
            Xmin = min.x + 0.15;
            Xmax = max.x + 0.05;
            pcl::PointCloud<PointT>::Ptr cloud_cylinder2(new pcl::PointCloud<PointT>());
            //找转轴的条件：近似与y轴平行；且两根圆柱中心之距离小于一定范围
            // 设置counter是为了避免陷入死循环，循环三次后即使没有正确结果也强制退出
            while ((abs(flag) > 0.3 || point_num == 0) && counter <= 1) // && abs(aa-a)>=0.3
            {                                                           //|| distance>=0.3
                pass2.setFilterLimits(Xmin, Xmax);                      //初始化带通滤波之上下限
                pass2.filter(*plus);

                ne2.setInputCloud(plus);
                ne2.compute(*cloud_normals2);

                seg2.setInputCloud(plus); //待分割点云
                seg2.setInputNormals(cloud_normals2);

                seg2.segment(*inliers_cylinder2, *coefficients_cylinder2);
                flag = coefficients_cylinder2->values[3];
                cout << counter << "正在搜索方向向量:(" << coefficients_cylinder2->values[3] << "," << coefficients_cylinder2->values[4] << "," << coefficients_cylinder2->values[5] << ")" << endl;
                aa = coefficients_cylinder2->values[0]; //转轴圆柱上一点坐标x
                bb = coefficients_cylinder2->values[1]; //转轴圆柱上一点坐标y
                cc = coefficients_cylinder2->values[2]; //转轴圆柱上一点坐标z
                // distance=abs(a-aa);
                counter++;
                Xmin = Xmin + Xdelta;
                Xmax = Xmax - Xdelta;

                pcl::ExtractIndices<PointT> extract3;   // 创建索引提取点对象
                extract3.setInputCloud(plus);           // 设置输入点云：待分割点云
                extract3.setIndices(inliers_cylinder2); // 设置内点索引
                extract3.setNegative(false);            // 默认false提取圆柱体内点
                // pcl::PointCloud<PointT>::Ptr cloud_cylinder2(new pcl::PointCloud<PointT>());
                extract3.filter(*cloud_cylinder2); // 执行滤波，并将结果点云保存到cloud_cylinder中
                point_num = cloud_cylinder2->points.size();
                // // 此处导出的点云应当为干净的转轴圆柱
            }
            cout << "OJBK2" << endl;
            {
                pcl::PCDWriter writer;
                writer.write<pcl::PointXYZ>("/home/aemc/catkin_ws/devel/lib/rvv/out/cylinder2.pcd", *cloud_cylinder2, false);
            }

            // pcl::ExtractIndices<PointT> extract3;   // 创建索引提取点对象
            // extract3.setInputCloud(plus);           // 设置输入点云：待分割点云
            // extract3.setIndices(inliers_cylinder2); // 设置内点索引
            // extract3.setNegative(false);                // 默认false提取圆柱体内点
            // pcl::PointCloud<PointT>::Ptr cloud_cylinder2(new pcl::PointCloud<PointT>());
            // extract3.filter(*cloud_cylinder2);      // 执行滤波，并将结果点云保存到cloud_cylinder中

            if (!cloud_cylinder2->points.empty())
            {
                // pcl::PCDWriter writer;
                // writer.write("out/cylinder2.pcd", *cloud_cylinder2, true);
                cout << "转轴轴线点坐标(" << coefficients_cylinder2->values[0] << "," << coefficients_cylinder2->values[1] << "," << coefficients_cylinder2->values[2] << ")" << endl;
                cout << "转轴方向向量(" << coefficients_cylinder2->values[3] << "," << coefficients_cylinder2->values[4] << "," << coefficients_cylinder2->values[5] << ")" << endl;
                cout << "转轴半径" << coefficients_cylinder2->values[6] << endl;

                End = clock();
                double endtime = (double)(End - Start) / CLOCKS_PER_SEC;
                cout << "Total time:" << endtime << endl;
                if (!talker(client, bool(type), float(distance), min.x, y_min, z_min, coefficients_cylinder2->values[0], coefficients_cylinder2->values[1], coefficients_cylinder2->values[2], coefficients_cylinder2->values[3], coefficients_cylinder2->values[4], coefficients_cylinder2->values[5]))
                {
                    goto L1;
                }

                close_fucking_rvc(x1);
            }
            else
            {
                PCL_ERROR("糟糕！未提取出转轴圆柱轴线！");
                End = clock();
                double endtime = (double)(End - Start) / CLOCKS_PER_SEC;
                cout << "Total time:" << endtime << endl;
                goto L1;
            }
            cout << "over OJBK" << endl;
        }
        else
        {
            PCL_ERROR("糟糕！抠掉把手后的点云聚类为空");
            goto L1;
        }
    }
    else
    {
        PCL_ERROR("糟糕！未提取出把手圆柱体！");
        goto L1;
    }

    return 1;
}
