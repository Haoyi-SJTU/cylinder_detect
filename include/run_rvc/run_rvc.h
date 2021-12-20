#include <RVC/RVC.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

RVC::X1 open_fucking_rvc() 
{
    FILE *DATA;
    double type = 0;
    // double plane_distance = 0;
    DATA = fopen("/home/aemc/catkin_ws/devel/lib/rvv/out/result.txt","r");
    if(DATA == NULL)
    {cout<<"妹找到txt文件啊"<<endl;}
    else
    {
        fscanf(DATA, "%lf", &type);
    } 

    RVC::SystemInit();// Initialize RVC X system.
    RVC::Device devices[10];// Scan all GigE RVC X Camera devices.
    size_t actual_size = 0;
    SystemListDevices(devices, 10, &actual_size, RVC::SystemListDeviceType::GigE);
    // Find whether any RVC X Camera is connected or not.
    if (actual_size == 0) 
    {
        std::cout << "Can not find any GigE RVC X Camera!" << std::endl;
        // return -1;
    }
        RVC::X1 x1;
    if(type)// 若type为1，则为上摘钩，使用右相机
    {
        x1 = RVC::X1::Create(devices[0], RVC::CameraID_Right);
        cout<<"进入上摘钩模式"<<endl;
    }
    else  // 若type为0，则为上摘钩，使用右相机
    {
       x1 = RVC::X1::Create(devices[0], RVC::CameraID_Left);
        cout<<"进入下摘钩模式"<<endl;
    }
    x1.Open();
    if (!x1.IsOpen()) 
    {
        std::cout << "没有打开相机!" << std::endl;
        RVC::X1::Destroy(x1);
        RVC::SystemShutdown();
        // return 1;
    }
    return x1;

}

int close_fucking_rvc(RVC::X1 &x1) 
{
    x1.Close();// Close RVC X Camera.
    RVC::X1::Destroy(x1);// Destroy RVC X Camera.
    RVC::SystemShutdown();// Shutdown RVC X System.
    return 0;
}


static pcl::PointCloud<pcl::PointXYZ>::Ptr PointMap2CloudPoint(RVC::PointMap &pm) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->height = pm.GetSize().height;
    cloud->width = pm.GetSize().width;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    const unsigned int pm_sz = cloud->height * cloud->width;
    const double *pm_data = pm.GetPointDataPtr();
    for (int i = 0; i < pm_sz; i++, pm_data += 6) 
    // for (int i = 0; i < pm_sz; i++, pm_data += 3)
    {
        
        cloud->points[i].x = pm_data[0];
        cloud->points[i].y = pm_data[1];
        cloud->points[i].z = pm_data[2];
    }
    return cloud;
}
