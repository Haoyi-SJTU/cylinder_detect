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


## 硬件操作步骤：

1. 首先检查电脑软硬件配置：
- 硬件要求：CPU: Intel i77代以上；内存：12G以上；**不建议使用虚拟机。**
- 软件要求：Ubuntu18以上，一般比最新版的晚1代即可；
2. 在[如本科技官网](https://rvbust.com/download.html/)下载最新版的RVC manager 
2. 安装，使用dpkg -i 或者双击安装包皆可，但要注意，部分电脑在安装时不会一次成功。安装后单击打开，如果左侧正常出现RVC图标（一个红色圆形，或者黑色圆形是他的logo没加载出来问题不大），但不能出现主界面，说明安装失败。首先检查电脑软硬件配置；反复安装、重启几次；如果还不行，可以拆下来硬盘换到另一台电脑上试一试。（啊这就涉及到如果你的bios分区设置的不好可能没法在别的电脑上启动，这就是另一个大坑了……）
一般来说，最近几代的RVC软件稳定性好了很多，不大可能出现因为软件的bug安装失败；安装g掉的原因大多是系统太老适配不了新版软件、和硬件配置太辣鸡；
4. 连接相机与电脑。可以使用网线直连或交换机中继。网线需要千兆的。交换机也是。连接、供电后，观察相机网线接口处，有发送和接收的两盏灯，一个橘色一个绿色（还是黄色？我记不清了），两盏灯都需要点亮。如果不亮，重新插拔网口、更换网线和交换机、重启相机。
5. 到这一步你已经可以启动RVC主界面了。连接相机，在软件上扫描接口。如果幸运，你可以扫描到相机此时左下角会出现相机IP设置按钮，点开，为相机左光机、右光机、机身分别指定IP，注意每指定一个IP都需要刷新一遍。等到IP地址不再报错，就可以打开相机了。
6. 打开相机，设置好曝光等参数，随便拍几张。**一般来说，只要在RVC软件里能正常拍照，你后面调用SDK包就没太大问题。**
7. 在主界面里找到例程，跳转，打开，修改IP，编译，运行。
8. enjoy it!
