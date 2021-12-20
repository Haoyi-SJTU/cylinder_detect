#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>					// 带通滤波器
#include <pcl/features/normal_3d.h>						// 法向量估计
#include <pcl/segmentation/sac_segmentation.h>			// 模型分割
#include <pcl/filters/extract_indices.h>				// 索引提取
#include <pcl/filters/statistical_outlier_removal.h>	// 点云离群值移除
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>						//体素
#include <pcl/segmentation/extract_clusters.h>			// 聚类算法
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include<ctime>
#include<cstdlib>
#include<cmath>
#include<iostream>
#include<stdio.h>
#include<ros/ros.h>

// #include<couple/a.h>
#include<std_msgs/String.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float32.h>

#include <couple/result.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>

using namespace std;
clock_t Start,End;

typedef pcl::PointXYZ PointT;


// 标定得到的手眼矩阵
float eye[4][4]=
	{{0.220678, 0.689145, -0.690172, 0.661388+0.1},
	{-0.221324, 0.724541, 0.652696, -0.0680546-0.02+0.13},
	{0.949901,0.00871628, 0.31243, -0.377535+0.02},
	{0,0,0,1}};
	// {{0.869150261985403, 0.2654677481802064, 0.4172585490638098, -0.006442770661442732},
 // {-0.2672550810194353, 0.9620336572143882, -0.05537114822725079, -0.1154695129642613},
 // {-0.4161160219938668, -0.06338861934795291, 0.907099409754609, -0.1404714404479622},
 // {0, 0, 0, 1}};

// float eye[4][4]=
// 	{{0.209582, 0.666278, -0.715615, 0.646779},
// 	{-0.257517, 0.743643, 0.616956, -0.0565715-0.02},//
// 	{0.943268,0.0549783, 0.327448, -0.392704+0.02},//
// 	{0,0,0,1}};

// 数组乘法计算
// 输入：eye手眼标定矩阵，point待变换的坐标
// 输出：result变换结果，以指针传递
int multi(float eye[4][4],float point[4][1],float result[4][1])
{
	int b = 0;
	for (int a = 0;a<4;++a)
	{
		result[a][0] = 0;
		for(int b =0;b<4;++b)
		{result[a][0]=result[a][0]+eye[a][b]*point[b][0];}
	}
	return 1;
}

// 发布自定义ROS消息
int talker(ros::ServiceClient client,bool type,float distance,float x,float y,float z,float a,float b,float c,float d,float e,float f)
{
	float m=a+d;
	float n=b+e;
	float p=c+f;
	float ab=sqrt((a-m)*(a-m)+(b-n)*(b-n)+(c-p)*(c-p));
	float ap=sqrt((a-x)*(a-x)+(b-y)*(b-y)+(c-z)*(c-z));
	float bp=sqrt((m-x)*(m-x)+(n-y)*(n-y)+(p-z)*(p-z));
	float cosa=(ap*ap+ab*ab-bp*bp)/(2*ab*ap);
	float sina=sqrt(1-cosa*cosa);
	// cout<<"distance"<<ap*sina<<endl;
	float point[4][1] = {{x},{y},{z},{1}};
	float result2[4][1] = {0};
	multi(eye, point,result2);

	float point0[4][1] = {{a},{b},{c},{1}};
	float result02[4][1] = {0};
	multi(eye, point0,result02);

	float point1[4][1] = {{d},{e},{f},{0}};
	float result12[4][1] = {0};
	multi(eye, point1,result12);

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
	cout<<result2[0][0]<<result2[1][0]<<result2[2][0]<<endl;
	cout<<result02[0][0]<<result02[1][0]<<result02[2][0]<<endl;
	cout<<result12[0][0]<<result12[1][0]<<result12[2][0]<<endl;
	while (!client.call(srv))
	{
        // cout<<"妹有发送成功"<<endl;
    }
    cout<<"result of planning"<<(bool)srv.response.status<<endl;
    return (bool)srv.response.status;
  }


int main(int argc, char **argv)
{
L1:
	Start=clock();	
	system("./PointCloud");//调用拍照程序

	End=clock();
	double endtime=(double)(End-Start)/CLOCKS_PER_SEC;
	cout<<"capture time:"<<endtime<<endl;

	// ros::init(argc, argv, "talker");
	// ros::NodeHandle node;
	// ros::ServiceClient client = node.serviceClient<couple::result>("talker");
	PCL_ERROR("毛泽东思想放光芒！");


	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PLYReader reader;
	reader.read("out/0.ply", *cloud);
	// 降采样
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.003, 0.003, 0.003);//单位是什么？(0.003f, 0.003f, 0.003f)
	sor.filter(*cloud);

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

	//平面的系数  ax + by + cz +d  =0;
	std::cerr << "平面方程: " 
		<<coefficients->values[0]<<"x+"<<coefficients->values[1]<< "y+"
		<<coefficients->values[2]<<"z+"<<coefficients->values[3]<<"=0"<<std::endl;
	// 计算当前识别的平面相对标准平面的偏差 -0.233338x+-0.698627y+0.676368z+-1.12356
	float aa=(coefficients->values[0]-0.233338)/2;
	float bb=(coefficients->values[1]-0.698627)/2;
	float cc=(coefficients->values[2]+0.676368)/2;
	float dd=coefficients->values[3]+1.12356;
	float distance=-dd/(sqrt(aa*aa+bb*bb+cc*cc));
	if(abs(distance) < 0.9)// 仅当当前平面平行于标准平面，且两者距离不超过0.9m时
	{
		cout<<"车车平移距离："<<distance<<endl;
	}

	pcl::ExtractIndices<PointT> extract00;	// 创建索引提取点对象
	extract00.setInputCloud(cloud);			// 设置输入点云：待分割点云
	extract00.setIndices(inliers);	// 设置内点索引
	extract00.setNegative(true);				// true，提取圆柱体外点
	// pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	extract00.filter(*cloud);		// 执行滤波，并将结果点云保存到cloud中


	{pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("out/clear.pcd", *cloud, false);}
	
	FILE *DATA;
	double type = 0;
	double plane_distance = 0;
	DATA = fopen("out/result.txt","r");
	if(DATA == NULL)
	{cout<<"妹找到txt文件啊"<<endl;}
	else
	{
		fscanf(DATA, "%lf", &type);
		// fscanf(DATA, "%lf,%lf", &type, &plane_distance);
	}

	// pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
	// pcl::PassThrough<pcl::PointXYZ> pass0;
	// pass0.setFilterFieldName ("y");
	// pass0.setInputCloud (cloud);
	// pass0.setFilterLimits (-0.8, 0.3);
	// pass0.filter (*cloud);//直接在输入点云cloud上裁剪

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName ("x");
	pass.setInputCloud (cloud);

	pcl::NormalEstimation<PointT, pcl::Normal> ne;//创建法向量估计对象
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);//设置搜索方式
	ne.setKSearch(50);//设置K近邻搜索点的个数
	
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;// 创建圆柱体分割对象
	seg.setOptimizeCoefficients(true);// 设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_CYLINDER);// 设置分割模型为圆柱体模型
	seg.setMethodType(pcl::SAC_RANSAC);// 设置采用RANSAC算法进行参数估计
	seg.setNormalDistanceWeight(0.016);	// 设置表面法线权重系数
	seg.setAxis({1,0,0});//z轴方向
	seg.setEpsAngle(0.8);//偏离角度（弧度制）
	seg.setMaxIterations(4000);// 设置迭代的最大次数
	seg.setDistanceThreshold(0.007);// 设置内点到模型距离的最大值
	seg.setRadiusLimits(0.01, 0.016);// 设置圆柱模型半径的范围
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);// 保存分割结果
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);// 保存圆柱体模型系数
	float flag=0;
	int counter=0;
	float Xmin=-0.5;
	float Xmax=0.5;
	float Xdelta=0.05;
	//条件：圆柱轴线近似平行于X轴。阈值：0.5
	// 设置counter是为了避免陷入死循环，循环三次后即使没有正确结果也强制退出
	while(abs(flag) < 0.6 && counter<=3)
		{
			// {pcl::PCDWriter writer;
			// 	writer.write<pcl::PointXYZ> ("out/fliter0.pcd", *cloud, false);}
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setFilterLimits (Xmin, Xmax);
			pass.filter (*cloud);//直接在输入点云cloud上裁剪
			ne.setInputCloud(cloud);//设置输入点云
			ne.compute(*cloud_normals);

			seg.setInputCloud(cloud);//待分割点云
			seg.setInputNormals(cloud_normals);
	    	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	    	flag=coefficients_cylinder->values[3];
	    	cout<<"正在搜索向量("<<coefficients_cylinder->values[3]<<","<<coefficients_cylinder->values[4]<<","<<coefficients_cylinder->values[5]<<")"<<endl;
	    	counter++;
	    	Xmin=Xmin+Xdelta;
	    	Xmax=Xmax-Xdelta;
	    }
	// 存放轴线函数的点斜式
	float a=float(coefficients_cylinder->values[0]);
	float b=float(coefficients_cylinder->values[1]);
	float c=float(coefficients_cylinder->values[2]);
	float d=float(coefficients_cylinder->values[3]);
	float e=float(coefficients_cylinder->values[4]);
	float f=float(coefficients_cylinder->values[5]);

	// cout<<"轴线点坐标("<<a<<","<<b<<","<<c<<")"<<endl;
	cout<<"把手方向向量("<<d<<","<<e<<","<<f<<")"<<endl;
	cout<<"把手半径"<<coefficients_cylinder->values[6]<<endl;

	// 抠出来把手圆柱点云
	pcl::ExtractIndices<PointT> extract;	// 创建索引提取点对象
	extract.setInputCloud(cloud);			// 设置输入点云：待分割点云
	extract.setIndices(inliers_cylinder);	// 设置内点索引
	extract.setNegative(false);				// 默认false，提取圆柱体内点；true，提取圆柱体外点
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	extract.filter(*cloud_cylinder);		// 执行滤波，并将结果点云保存到cloud_cylinder中

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	if (!cloud_cylinder->points.empty())//如果把手点云非空
	{

		// 点云离群值移除
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
		sor2.setInputCloud(cloud_cylinder);
		sor2.setMeanK(5);//临近点
		sor2.setStddevMulThresh(1);//距离大于1倍标准方差
		sor2.filter(*cloud_cylinder);

		pcl::PointXYZ min0;//用于存放三个轴的最小值
		pcl::PointXYZ max0;//用于存放三个轴的最大值
		pcl::getMinMax3D(*cloud_cylinder,min0,max0);
		pcl::PointCloud<pcl::PointXYZ> copy0 ;
		copy0 = *cloud;// 原始点云的深拷贝
		pcl::PointCloud<PointT>::Ptr copy(new pcl::PointCloud<PointT>);
		*copy = copy0;//建立拷贝点云的指针
		pcl::PassThrough<pcl::PointXYZ> pass0;
		pass0.setFilterFieldName ("x");
		pass0.setInputCloud (copy);
		pass0.setFilterLimits (min0.x-0.05, max0.x+0.05);
		pass0.filter (*copy);//直接在输入点云上裁剪

		pass0.setFilterFieldName ("y");
		pass0.setInputCloud (copy);
		pass0.setFilterLimits (min0.y-0.03, max0.y+0.02);
		pass0.filter (*copy);//直接在输入点云cloud上裁剪

		// cout<<copy->points.size()<<endl;
		// {pcl::PCDWriter writer;
		// writer.write<pcl::PointXYZ> ("out/copy.pcd", *copy, false);}

		//点云欧式聚类
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
		ece.setInputCloud(copy);
		ece.setClusterTolerance(0.005);
		ece.setMinClusterSize(20);
		ece.setMaxClusterSize(8000);
		pcl::search::KdTree<PointT>::Ptr tree3(new pcl::search::KdTree<PointT>());
		ece.setSearchMethod(tree3);
		ece.extract(cluster_indices);

		int max_point = 0;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cut_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
		// 遍历各个聚类结果，找出来点数最大的，认为是纯粹的把手圆柱
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				cloud_cluster->push_back ((*copy)[*pit]);
			cloud_cluster->width = cloud_cluster->size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			if(cloud_cluster->size () > max_point)
				{
					*cut_cylinder = (*cloud_cluster);
					max_point = cloud_cluster->size();
					cout<<cut_cylinder->points.size()<<endl;
				}
		}

		// cout<<cut_cylinder->points.size()<<endl;
		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ> ("out/cylinder.pcd", *cut_cylinder, false);
		// 计算圆柱轴线两个端点的坐标
		pcl::PointXYZ min;//用于存放三个轴的最小值
		pcl::PointXYZ max;//用于存放三个轴的最大值
		pcl::getMinMax3D(*cut_cylinder,min,max);
		float y_min = (min.x-a)*e/d + b;
		float z_min = (min.x-a)*f/d + c;
		cout<<"呐呐呐把手下端点("<<min.x<<","<<y_min<<","<<z_min<<")"<<endl;
		
		// 找转轴//////////////////////
		// 去掉已经识别出来的圆柱
		pcl::ExtractIndices<PointT> extract2;	//创建索引提取点对象
		extract2.setInputCloud(cloud);			//输入：待分割点云
		extract2.setIndices(inliers_cylinder);	//设置内点索引
		extract2.setNegative(true);				//true，提取圆柱体外点
		pcl::PointCloud<PointT>::Ptr rest(new pcl::PointCloud<PointT>());
		extract2.filter(*rest);

		// 对抠出来的把手做离群值移除
		// 为了排除少部分离散的点对找圆柱上下边界之影响
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor3;
		sor3.setInputCloud(rest);
		sor3.setMeanK(30);//50个临近点
		sor3.setStddevMulThresh(1.0);//距离大于1倍标准方差
		sor3.filter(*rest);

/*		//点云欧式聚类
		std::vector<pcl::PointIndices> cluster_indices2;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece2;
		ece2.setInputCloud(rest);
		ece2.setClusterTolerance(0.02);
		ece2.setMinClusterSize(2);
		ece2.setMaxClusterSize(3000);
		ece2.setSearchMethod(tree);
		ece2.extract(cluster_indices2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster1 (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr plus (new pcl::PointCloud<pcl::PointXYZ>);
		
		// 遍历各个聚类结果，找出来点数大于50小于500 的，把把手残留的边边角角抠干净
		// 因为转轴圆柱遮挡很多，聚类结果可能会被分成多个类，所以这里只过滤掉点数明显过多或过少的类，保留可能不止一个的正确类
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices2.begin (); it != cluster_indices2.end (); ++it)
		{
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{cloud_cluster1->push_back ((*rest)[*pit]); }
			cloud_cluster1->width = cloud_cluster1->size ();
			cloud_cluster1->height = 1;
			cloud_cluster1->is_dense = true;
			if(cloud_cluster1->size () <= 20 || cloud_cluster1->size () >= 3000)
				continue;
			*plus = (*plus) + ( *cloud_cluster1 );
			// j++;
		}

		{pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ> ("out/rest.pcd", *plus, false);}
*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr plus (new pcl::PointCloud<pcl::PointXYZ>);
		*plus = *rest;
		if (!plus->points.empty())//如果把手点云非空
		{
			// 此处导出的点云应当为转轴圆柱，可能有一段把手到转轴的圆弧过渡
			

			pcl::PassThrough<pcl::PointXYZ> pass2;
			pass2.setFilterFieldName ("x");
			pass2.setInputCloud (plus);//直接在输入点云cloud上裁剪

			pcl::NormalEstimation<PointT, pcl::Normal> ne2;//创建法向量估计对象
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<PointT>::Ptr tree2(new pcl::search::KdTree<PointT>());
			ne2.setSearchMethod(tree2);//设置搜索方式
			ne2.setKSearch(50);//设置K近邻搜索点的个数
			ne2.setInputCloud(plus);//设置输入点云

			pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg2;// 创建圆柱体分割对象
			seg2.setOptimizeCoefficients(true);// 设置对估计的模型系数需要进行优化
			seg2.setModelType(pcl::SACMODEL_CYLINDER);// 设置分割模型为圆柱体模型
			seg2.setMethodType(pcl::SAC_RANSAC);// 设置采用RANSAC算法进行参数估计
			seg2.setNormalDistanceWeight(0.016);	// 设置表面法线权重系数
			seg2.setMaxIterations(900);// 设置迭代的最大次数
			seg2.setAxis({0,1,0});//z轴方向
			seg2.setEpsAngle(0.9);//偏离角度（弧度制）
			seg2.setDistanceThreshold(0.007);// 设置内点到模型距离的最大值
			seg2.setRadiusLimits(0.009, 0.023);// 设置圆柱模型半径的范围
			pcl::PointIndices::Ptr inliers_cylinder2(new pcl::PointIndices);// 保存分割结果
			pcl::ModelCoefficients::Ptr coefficients_cylinder2(new pcl::ModelCoefficients);// 保存圆柱体模型系数
			flag=0;
			counter=0;
			// float distance=1;//两圆柱中心距离之平方
			float aa=0;
			float bb=0;
			float cc=0;
			Xdelta = 0.05;
			Xmin = min.x+0.1;
			Xmax = max.x+0.1;
			//找转轴的条件：近似与y轴平行；且两根圆柱中心之距离小于一定范围
			// 设置counter是为了避免陷入死循环，循环三次后即使没有正确结果也强制退出
			while( (abs(flag)<0.5 || abs(aa-min.x)>=0.5) && counter<=1 )// && abs(aa-a)>=0.3
			{//|| distance>=0.3 
				pass2.setFilterLimits (Xmin, Xmax);//初始化带通滤波之上下限
				pass2.filter (*plus);

			// 	{pcl::PCDWriter writer;
			// writer.write<pcl::PointXYZ> ("out/fliter.pcd", *plus, false);}

				ne2.compute(*cloud_normals2);
				seg2.setInputCloud(plus);//待分割点云
				seg2.setInputNormals(cloud_normals2);
	    		seg2.segment(*inliers_cylinder2, *coefficients_cylinder2);
	    		flag=coefficients_cylinder2->values[4];
	    		cout<<"正在搜索方向向量:("<<coefficients_cylinder2->values[3]<<","<<coefficients_cylinder2->values[4]<<","<<coefficients_cylinder2->values[5]<<")"<<endl;
	    		aa=coefficients_cylinder2->values[0];//转轴圆柱上一点坐标x
				bb=coefficients_cylinder2->values[1];//转轴圆柱上一点坐标y
				cc=coefficients_cylinder2->values[2];//转轴圆柱上一点坐标z
				// distance=abs(a-aa);
				counter++;
				Xmin=Xmin+Xdelta;
				Xmax=Xmax-Xdelta;
				// // 此处导出的点云应当为干净的转轴圆柱
				//{pcl::PCDWriter writer;
				// writer.write<pcl::PointXYZ> ("out/fliter2.pcd", *plus, false);}
			}

			pcl::ExtractIndices<PointT> extract3;	// 创建索引提取点对象
			extract3.setInputCloud(plus);			// 设置输入点云：待分割点云
			extract3.setIndices(inliers_cylinder2);	// 设置内点索引
			extract3.setNegative(false);				// 默认false提取圆柱体内点
			pcl::PointCloud<PointT>::Ptr cloud_cylinder2(new pcl::PointCloud<PointT>());
			extract3.filter(*cloud_cylinder2);		// 执行滤波，并将结果点云保存到cloud_cylinder中

			if (!cloud_cylinder2->points.empty())
			{	
				pcl::PCDWriter writer;
				writer.write("out/cylinder2.pcd", *cloud_cylinder2, true);
				cout<<"转轴轴线点坐标("<<coefficients_cylinder2->values[0]<<","<<coefficients_cylinder2->values[1]<<","<<coefficients_cylinder2->values[2]<<")"<<endl;
				cout<<"转轴方向向量("<<coefficients_cylinder2->values[3]<<","<<coefficients_cylinder2->values[4]<<","<<coefficients_cylinder2->values[5]<<")"<<endl;
				cout<<"转轴半径"<<coefficients_cylinder2->values[6]<<endl;

				End=clock();
				double endtime=(double)(End-Start)/CLOCKS_PER_SEC;
				cout<<"Total time:"<<endtime<<endl;
				// if (! talker(client,bool(type),float(distance),min.x,y_min,z_min,coefficients_cylinder2->values[0],coefficients_cylinder2->values[1],coefficients_cylinder2->values[2],coefficients_cylinder2->values[3],coefficients_cylinder2->values[4],coefficients_cylinder2->values[5]))
				// {
				// 	// goto  L1;
				// }
			}
			else
			{
				PCL_ERROR("糟糕！未提取出转轴圆柱轴线！");
					End=clock();
	double endtime=(double)(End-Start)/CLOCKS_PER_SEC;
	cout<<"Total time:"<<endtime<<endl; 
				goto L1;
			}
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
	return 0;
}

