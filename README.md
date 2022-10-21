目录
- [简述：](#简述)
- [代码内容：](#代码内容)
- [函数、变量说明：](#函数变量说明)
- [使用设备：](#使用设备)
- [硬件操作步骤：](#硬件操作步骤)
- [函数流程：](#函数流程)
    - [1. 检查点云输出文件夹是否存在。](#1-检查点云输出文件夹是否存在)
    - [2. 读取车号识别结果。](#2-读取车号识别结果)
    - [3. 根据车钩类型调用相机对象创建函数。](#3-根据车钩类型调用相机对象创建函数)
    - [4. 设置相机曝光时间。](#4-设置相机曝光时间)
    - [5. 记录拍照和发送ROS消息的次数.](#5-记录拍照和发送ros消息的次数)
    - [6. 初始化ROS节点和服务发布。](#6-初始化ros节点和服务发布)
    - [7. 拍照，或者调用存储的点云。](#7-拍照或者调用存储的点云)
    - [8. 判断拍照结果是否为空。](#8-判断拍照结果是否为空)
    - [9. 删除无效点。](#9-删除无效点)
    - [10. 带通滤波清理Y方向的点。](#10-带通滤波清理y方向的点)
    - [11. 调用PCL库，提取平面。](#11-调用pcl库提取平面)
    - [12. 使用PCL库分割点云。](#12-使用pcl库分割点云)
    - [13. 去除梯子。](#13-去除梯子)
    - [14. 带通滤波沿X方向清理截断。](#14-带通滤波沿x方向清理截断)
    - [15. 边角料清理。](#15-边角料清理)
    - [16. 聚类清理。](#16-聚类清理)
    - [17. 使用PCL库函数，RANSAC方法寻找圆柱。](#17-使用pcl库函数ransac方法寻找圆柱)
    - [18. 洛阳铲。](#18-洛阳铲)
    - [19. 聚类提取完整圆柱。](#19-聚类提取完整圆柱)
    - [20. 存储去掉把手后的点云，用于规划器避障。](#20-存储去掉把手后的点云用于规划器避障)
    - [21. 从点云图中去掉把手，并清理把手附近的噪点。](#21-从点云图中去掉把手并清理把手附近的噪点)
    - [22. 同上，再次找出转轴圆柱。](#22-同上再次找出转轴圆柱)
    - [23. 同上，记录转轴圆柱数据。](#23-同上记录转轴圆柱数据)
    - [24. 发布ROS服务消息，根据返回的规划结果（成功/失败）写入状态。](#24-发布ros服务消息根据返回的规划结果成功失败写入状态)
    - [25. 关闭相机。](#25-关闭相机)
    - [26. Case1: 转轴提取失败](#26-case1-转轴提取失败)
    - [27. Case2: 转轴没了](#27-case2-转轴没了)
    - [28. Case3: 连续三次啥也没找到](#28-case3-连续三次啥也没找到)


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

## 函数流程：

主函数流程简述。


```mermaid
graph TB
    XX(准备) --> X[读车号]
    X[读车号] --> XXX[拍照]
    XXX[拍照] -->  Q{拍照结果为空?}
    Q{拍照结果为空?} --否--> A[找平面]
    Q{拍照结果为空?} --是--> XXX[拍照]
    A[找平面] --> --> B[平面移除]
    B[平面移除] --> C[清理杂点]
    C[清理杂点] --> D[找把手圆柱]
    D[找把手圆柱] --> E{找到把手圆柱?}
    E{找到把手圆柱?} --是--> F[提取完整圆柱]
    E{找到把手圆柱?} --否--> XXX[拍照]
    F[提取完整圆柱] --> G[存储避障点云]
    G[存储避障点云] --> H[清理杂点]
    H[清理杂点] --> I[找转轴圆柱]
    I[找转轴圆柱] --> J{找到转轴圆柱?}
    J{找到转轴圆柱?} --是--> K[提取完整圆柱]
    J{找到转轴圆柱?} --否--> XXX[拍照]
    K[提取完整圆柱] --> L[发布ROS服务消息]
    L[发布ROS服务消息] --> M[返回规划结果]
    M[返回规划结果] --> N{规划成功?}
    N{规划成功?} --是--> O(结束)
    N{规划成功?} --否--> P{已经重复了3次?}
    P{已经重复了3次?}--是--> O(结束)
    P{已经重复了3次?}--否-->  XXX[拍照]
```


#### 1. 检查点云输出文件夹是否存在。

若不存在则建立。
```
    DIR *dir;
    if ((dir = opendir("/home/aemc/catkin_ws/devel/lib/rvv/out/")) == NULL)
    {
        system("mkdir -p /home/aemc/catkin_ws/devel/lib/rvv/out/");
    }
```

#### 2. 读取车号识别结果。

按照结果得到type量。
上摘钩：type=1;
下摘钩：type=0;
根据tyoe决定使用相机的哪个镜筒。这个量需要留存，后面选择手眼标定矩阵和ROS消息发布都要用到。

```
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
```

#### 3. 根据车钩类型调用相机对象创建函数。

```
    RVC::X1 x1 = open_rvc(type);
```

#### 4. 设置相机曝光时间。

不同的光照条件决定相机曝光参数。这里简单地根据当前时间选择。在实际测试中，车钩基本不反光，因此可以选择较大的曝光参数而不必担心会过曝。

```
    time_t nowtime;
    struct tm *p; //时间结构体指针 p
    time(&nowtime);
    p = localtime(&nowtime); //调用本地时间函数localtime将nowtime变量的日历时间转化为本地时间，存入指针p
    int hour = p->tm_hour;   // 在上午9点到下午4点之间3D曝光时间18%，中午11点到14点曝光时间50%，其它时间70%
    RVC::X1::CaptureOptions temp;
    if (hour >= 9 && hour < 16)
    {
        temp.exposure_time_3d = 70;
        if (hour >= 11 && hour <= 14)
            temp.exposure_time_3d = 50;
    }
    else
    {
        temp.exposure_time_3d = 70;
    }
```

#### 5. 记录拍照和发送ROS消息的次数.

如果多次拍照（说明前面几次拍照都识别不了）或多次发送（说明前面几次规划器规划失败），则认为无法摘钩。

```
    unsigned int times_of_capture = 0; //记录拍照次数
    unsigned int sent_srv_times = 0;   //记录发送srv的次数
```
#### 6. 初始化ROS节点和服务发布。

```
    ros::init(argc, argv, "talker");
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<couple::result>("talker");
    ros::ServiceClient collision_client = node.serviceClient<std_srvs::Empty>("collision_pc");
```

#### 7. 拍照，或者调用存储的点云。

```
    Start = clock();
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
#ifdef REAL_CLOUD
    if (x1.Capture(temp) == true)
    {
        std::cout << "在拍了在拍了!" << std::endl;
        RVC::PointMap pm = x1.GetPointMap();
        cloud = PointMap2CloudPoint(pm);
        End = clock();
        times_of_capture++;
    }
    else
    {
        std::cout << "拍照失败!" << std::endl;
    }
#endif

#ifdef TEST_CLOUD
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/aemc/catkin_ws/devel/lib/rvv/out/original.pcd", *cloud) == -1)
    {
        PCL_ERROR("测试点云读取失败! \n");
        return 0;
    }
    times_of_capture++;
#endif

    End = clock();
    double endtime = (double)(End - Start) / CLOCKS_PER_SEC;
    cout << "capture time:" << endtime << endl;
```

#### 8. 判断拍照结果是否为空。
一般有几种情况会空：火车仍在运动、距离太近或太远、过曝（基本不发生）、相机偶发故障。
```
    if (cloud->points.empty())
    {
        PCL_ERROR("点云为空，即将重新拍照…");
        goto L1;
    }
```

#### 9. 删除无效点。
这一步使用RVCmanager软件会帮你完成，但调用SDK拍照的话就需要自己去做了。
```
    // 删除点云无效点
    // cloud_in->is_dense = false
    // 为了防止去除点云无效值失败，手动设置点云标记为“存在无效值”
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
```

#### 10. 带通滤波清理Y方向的点。
这里Y方向就是水平面内近似垂直于铁轨方向，在这个方向上过于靠近铁轨内侧（火车钩舌）、过于靠近铁轨外侧（车厢外侧板）的点都要删去。
```

    pcl::PassThrough<pcl::PointXYZ> pass0;
    pass0.setFilterFieldName("y");
    pass0.setInputCloud(cloud);
    // pass0.setFilterLimits(-0.8, 0.6);
    pass0.setFilterLimits(0.0, 0.6);
    pass0.filter(*cloud); //直接在输入点云cloud上裁剪
```

#### 11. 调用PCL库，提取平面。
这里的平面就是每节车厢端面。
```
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

```

计算当前识别的平面相对标准平面的偏差。
标准平面来自于我们预先找一个停靠比较准确的车厢，运行程序得到车厢平面的方程。
我们默认两个平面相互平行，计算平面距离。
```
    write_txt(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    float aa = ((coefficients->values[0]) + 0.0912559) / 2;
    float bb = ((coefficients->values[1]) - 0.625265) / 2;
    float cc = ((coefficients->values[2]) + 0.775058) / 2;
    float dd = coefficients->values[3] + 1.10064;
    float distance = dd / (sqrt(aa * aa + bb * bb + cc * cc)); //distance=-dd/(sqrt(aa*aa+bb*bb+cc*cc));
```

如果距离差距不大，则可以摘钩。
否则重新拍照。（没什么L用）

```
    if (abs(distance) < 0.9)                                   // 仅当当前平面平行于标准平面，且两者距离不超过0.9m时
    {
        cout << "车车平移距离：" << distance << endl;
    }
    else
    {
        PCL_ERROR("车车位置超限，即将重新拍照…");
        // goto L1;
    }
```

#### 12. 使用PCL库分割点云。
这里把车厢平面、以及平面附近5cm内的点云全部移除掉，缩小点云规模。
把移除平面后的点云保存在指针**cloud2**中。
把移除的平面点云保存在指针**plane**中。

```
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract00; // 创建索引提取点对象
    extract00.setInputCloud(cloud);        // 设置输入点云：待分割点云
    extract00.setIndices(inliers);         // 设置内点索引
    extract00.setNegative(true);           // true，提取平面外点
    extract00.filter(*cloud2);             // 执行滤波，并将结果点云保存到cloud中

    pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>);
    extract00.setNegative(false); // flase,提取平面内的点
    extract00.filter(*plane);     // 执行滤波，并将结果点云保存到cloud中
    // 分割指定阈值内的平面
    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    double D = coefficients->values[3];
    vector<int> pointIdxVec;
    // 移除所有紧邻这个平面的点
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
```

存储去掉平面的点云。

```
    {
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ>("/home/aemc/catkin_ws/devel/lib/rvv/out/plane.pcd", *plane, false);
    }
```
此时我们的点云应该是没有了火车端面、在空中漂浮着的一个完整的车钩、和它的把手、以及**一些乱七八糟的梯子头**、以及**一部分车厢体的板子、车轮等**啥的。

#### 13. 去除梯子。
我们注意到，对于及**一些乱七八糟的梯子头**，它们往往位于车钩的正上方或正下方，并且由于梯子头的遮挡，在梯子头的地方相机看不到前述车厢截面的平面。
不严谨地说，我们看到的平面应该是完全笼罩了车钩。或者说，只有在车钩的这一片区域我们才能看到车厢截面的平面。
反过来说，**但凡被梯子挡住、看不到截面平面的地方，一定不是车钩。**
所以，我们再次利用前面提取出来的平面的信息。以前面提取出来的平面为边界，在竖直方向上删除所有高于平面最高点、低于平面最低点的点。

```
    pcl::PointXYZ plane_min; //用于存放三个轴的最小值
    pcl::PointXYZ plane_max; //用于存放三个轴的最大值
    pcl::getMinMax3D(*plane, plane_min, plane_max);
    pcl::PassThrough<pcl::PointXYZ> plane_y_filter;
    plane_y_filter.setFilterFieldName("y");
    plane_y_filter.setInputCloud(cloud2);
    plane_y_filter.setFilterLimits(plane_min.y - 0.05, 5);
    plane_y_filter.filter(*cloud2); //直接在输入点云cloud上裁剪
    plane_y_filter.setFilterFieldName("z");
    plane_y_filter.setInputCloud(cloud2);
    plane_y_filter.setFilterLimits(plane_min.z - 0.2, 5);
    plane_y_filter.filter(*cloud2); //直接在输入点云cloud上裁剪
```

#### 14. 带通滤波沿X方向清理截断。
沿X轴截断。相机X轴就是真实世界里竖直方向，这里因为车钩基本位于相机视野的中央（需要区分上下车钩），太靠上、太靠下的地方可以直接裁掉。

```
   if (type) // 若type为1，则为上摘钩，删除x坐标<0的点
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("x");
        pass.setInputCloud(cloud2);
       // pass.setFilterLimits(-0.6, 0.1);
        pass.setFilterLimits(-0.6, 0.4);
        pass.filter(*cloud2); //直接在输入点云cloud上裁剪
    }
    else // 若type为0，则为下摘钩，删除x坐标<0的点
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("x");
        pass.setInputCloud(cloud2);
        pass.setFilterLimits(-0.9, 0.2);
        pass.filter(*cloud2); //直接在输入点云cloud上裁剪
    }    

```

#### 15. 边角料清理。
经过前面大刀阔斧的裁剪后，剩下了很多边角料，反映在点云图像上就是在裁剪边界附近很多不构成形状的零碎噪点，来一波均值滤波直接干净。

```
    // 做圆柱拟合前先清理干净噪点
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
        sor2.setInputCloud(cloud2);
        sor2.setMeanK(5);             //临近点
        sor2.setStddevMulThresh(0.5); //距离大于1倍标准方差
        sor2.filter(*cloud2);
    }
```

#### 16. 聚类清理。
步骤15过后，点云干净了吗？没有。
一些大块的、构成形状的点，不能用滤波去掉，所以要再来一个聚类，把聚类后点数较小的块删掉。

```
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
```

可以想象，此时的点云应该只有漂浮在空中的一个车钩了。

#### 17. 使用PCL库函数，RANSAC方法寻找圆柱。
在搜索之前，我们会先设置一个沿竖直方向的带通滤波器。每经过一轮搜索，我就在竖直方向上把点云上边缘、下边缘剪掉一点，由此点云会逐步地向包含着车钩的中间区域逼近。

```
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
```

圆柱拟合的参数设置：`seg.setAxis({1, 0, 0});`和`seg.setEpsAngle(0.5);`决定了我们的搜索只会在竖直方向，只会找到车钩竖直方向的那个把手。

```
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
```

#### 18. 洛阳铲。
事实上，RANSAC提取的圆柱很容易漏掉一部分。
比如说一个圆柱它可能只找到了圆柱的下半部分，即使上半部分和它找到的下半部分挨得很近，也会因为一些原因找不到（阈值设置、或者单纯是RANSAC随机采样的原因）。
就好比去挖萝卜，我们找到了萝卜，但是并不知道萝卜到底有多长，可能一铲子下去萝卜就从中间断开了，不能完整地把萝卜挖出来。
所以，我们结合前面RANSAC找出来的圆柱，和我们的经，把圆柱及圆柱附近的一个长方形空间内的点全部拿出来。这个空间内可能会有点大，但圆柱把手一定是完整的。
就好比我们先用**洛阳铲**挖一个包着萝卜的土方出来，这里面可能会多出来一些泥土，但萝卜一定是完整的。


```
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
```
这样，我们挖出来了一个长方体的点云空间，这里面就有我们要找的、完整的把手圆柱。

#### 19. 聚类提取完整圆柱。

考虑到在这个长方体空间内，只有圆柱是最大的形状体，其余的都是些边边角角的小块。所以使用聚类算法找出其中最大的一部分，就是圆柱。

```
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
```
存储识别到的圆柱。实际使用时注释掉这一段以加快速度。
```
        // 存储识别到的把手
        {
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("/home/aemc/catkin_ws/devel/lib/rvv/out/cylinder.pcd", *cut_cylinder, false);
        }
        cout << "cylinder OJBK" << endl;
```

记录圆柱的上下端点。

```
        // 计算圆柱轴线两个端点的坐标
        pcl::PointXYZ min; //用于存放三个轴的最小值
        pcl::PointXYZ max; //用于存放三个轴的最大值
        pcl::getMinMax3D(*cut_cylinder, min, max);
        float x_min = min.x;
        float y_min = (min.x - a) * e / d + b;
        float z_min = (min.x - a) * f / d + c;
        float y_max = (max.x - a) * e / d + b;
        float z_max = (max.x - a) * f / d + c;
        cout <<"变换前的下坐标 "<< x_min <<" "<<y_min<<" "<<z_min<< endl
```

#### 20. 存储去掉把手后的点云，用于规划器避障。

```
        {
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("/home/aemc/catkin_ws/devel/lib/rvv/out/collision.pcd", *non_overlapA, false);
            std_srvs::Empty emp;
            collision_client.call(emp);
        }
```

#### 21. 从点云图中去掉把手，并清理把手附近的噪点。

```
        // //去掉已经识别出来的圆柱
        pcl::PointCloud<PointT>::Ptr plus(new pcl::PointCloud<PointT>());
        plus = cut_pointcloud(cloud2, cut_cylinder);

        // 对抠掉把手的剩余点云做离群值移除
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor3;
        sor3.setInputCloud(plus);
        sor3.setMeanK(30);            //50个临近点
        sor3.setStddevMulThresh(1.0); //距离大于1倍标准方差
        sor3.filter(*plus);
```
此时的点云应该是漂浮在空中的一截转轴圆柱。我们下面要找这个圆柱。

#### 22. 同上，再次找出转轴圆柱。
需要注意的是：转轴是垂直于把手的圆柱，在设置RANSAC参数时需要修改。

```
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
            Xmin = min.x + 0.2;
            Xmax = max.x + 0.1;
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

```

#### 23. 同上，记录转轴圆柱数据。

```
                cout << "转轴轴线点坐标(" << coefficients_cylinder2->values[0] << "," << coefficients_cylinder2->values[1] << "," << coefficients_cylinder2->values[2] << ")" << endl;
                cout << "转轴方向向量(" << coefficients_cylinder2->values[3] << "," << coefficients_cylinder2->values[4] << "," << coefficients_cylinder2->values[5] << ")" << endl;
                cout << "转轴半径" << coefficients_cylinder2->values[6] << endl;

```

#### 24. 发布ROS服务消息，根据返回的规划结果（成功/失败）写入状态。

```
    int planning_status = talker(client, bool(type), float(distance), x_min, y_min, z_min, coefficients_cylinder2->values[0], coefficients_cylinder2->values[1], coefficients_cylinder2->values[2], coefficients_cylinder2->values[3], coefficients_cylinder2->values[4], coefficients_cylinder2->values[5]);
    if (sent_srv_times <= 2 && !planning_status)
    {
        sent_srv_times++;
         goto L1;
    }
    if (planning_status)
    {
        write_status(1);
    }
    else
    {
        write_status(3);
    }
```

#### 25. 关闭相机。

```
    close_rvc(x1);
```

#### 26. Case1: 转轴提取失败
把手的提取成功率高于圆柱。在把手提取出来、但转轴提取失败的情况下，为了提高摘钩数量，我们人工写一个转轴参数进去。
以下代码是在转轴提取失败时调用。

```
    PCL_ERROR("识别错误！未提取出转轴圆柱轴线！");
    End = clock();
    double endtime = (double)(End - Start) / CLOCKS_PER_SEC;
    cout << "Total time:" << endtime << endl;
    if (times_of_capture > 3)
    {
        int planning_status;
        close_rvc(x1);
         if (type)
        {
            planning_status = talker(client, bool(type), float(distance), x_min, y_min, z_min, max.x, y_max, z_max, 0.007522, 0.784111, 0.6205748);
        }
        else
        {
            planning_status = talker(client, bool(type), float(distance), x_min, y_min, z_min, max.x, y_max, z_max, -0.12, 0.79408, 0.5958); //-0.12, 0.049, 1.228
        }
        if (planning_status)
        {
            write_status(1);
        }
        else
        {
            write_status(3);
        }
        return 0;
    }
```

#### 27. Case2: 转轴没了
接步骤26，更离谱的情况是，有些时候转轴圆柱整个都没了。
可能是拍到的转轴太小，被当成噪点删掉了。这个时候直接~~瞎编~~根据经验手写写一个参数进去。

```
if (times_of_capture > 3)
            {
                int planning_status;
                close_rvc(x1);
                if (type)
                {
                    planning_status = talker(client, bool(type), float(distance), min.x, y_min, z_min, max.x, y_max, z_max, 0.007522, 0.784111, 0.6205748);
                }
                else
                {
                    planning_status = talker(client, bool(type), float(distance), min.x, y_min, z_min, max.x, y_max, z_max, -0.12, 0.79408, 0.5958);
                }
                if (planning_status)
                {
                    write_status(1);
                }
                else
                {
                    write_status(3);
                }
                return 0;
            }
            goto L1;
        }
```

#### 28. Case3: 连续三次啥也没找到

接步骤27，最离谱的情况是，连续三次啥也没找到，或者连续三次规划器全部规划失败。

~~（三回啊三回）~~

直接写0发出去，这样规划器返回规划失败。

```
    PCL_ERROR("识别错误！未提取出把手圆柱体！");
    if (times_of_capture > 3)
    {
        close_rvc(x1);
        talker(client, bool(type), float(distance), 0, 0, 0, 0, 0, 0, 0, 0, 0);
        write_status(3);
        return 0;
    }
    goto L1;
    }
```
