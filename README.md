## 简述：

本代码用于车钩识别，为使用机械臂的自动化摘取车钩做准备。

## 代码内容：

基于C++版本的PCL库开发，基本思路是使用双目相机采集点云数据，点云经多级带通滤波、降采样、聚类后，排除掉目标点云附近的干扰物，对结果做基于RANSAC的圆柱拟合，得到把手的位姿：之后，去掉把手点云，对剩余的点云重复上述步骤，再次做基于RANSAC的圆柱拟合，得到转轴的位姿。综合把手和转轴的位姿数据即可得到车钩的全部位姿数据。


所用到的PCL库函数包括：

点云滤波：
    extrA.setInputCloud(cloud);                                       //设置输入点云
    
    extrA.setIndices(boost::make_shared<vector<int>>(pointIdxVec_A)); //设置索引
    extrA.setNegative(true);                                          // 提取对应索引之外的点
    extrA.filter(*non_overlapA);



## 函数、变量说明：

 - **float eye[4][4]**
   
   全局数组，4*4矩阵，记录手眼标定结果

 - **int multi(float eye[4][4], float point[4][1], float result[4][1])**

   作用：数组乘法计算

   输入：eye手眼标定矩阵，point待变换的坐标
   
   输出：result变换结果，以指针传递
   
 - **int talker(ros::ServiceClient client, bool type, float distance, float x, float y, float z, float a, float b, float c, float d, float e, float f)**

   作用：发布自定义ROS消息

   输入：所需发布的各种数据
  
   返回：发布状态值

 - **pcl::PointCloud<pcl::PointXYZ>::Ptr cut_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cylinder)**

   布尔运算：计算一个点云相比于两一个点云所独有的部分
   
   输入:源点云和剪刀点云，
   
   输出:源点云中去除和剪刀重叠部分的点云指针

 - **int main(int argc, char **argv)**
  
   主函数。完成双目相机数据采集、运行PCL识别、发布识别数据等操作。 

## 使用设备：

- 如本RVC双目结构光传感器
- 千兆以太网交换机
- 千兆网线
- 计算机（CPU: Intel 7代i7，内存：16G，硬盘：256G固态）

