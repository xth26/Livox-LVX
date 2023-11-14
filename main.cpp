#include<iostream>
#include<fstream>  //读取流的头文件

#include <cstdio>   //获取文件大小要用
#include <string>
#include <array> 

#include<pcl/io/pcd_io.h>	//PCD读写类相关的头文件
#include<pcl/point_types.h> //pcl中支持的点类型的头文件

#include"LivoxFileReader.h"

int main()
{	
	//lvx文件路径
	std::string path_lvx = "D:\\LAS_Files\\lvx_files\\2022_10_30.lvx";
	
	//点云数据
	std::vector<livox_ros_driver::CustomMsg> lidarDataVec;

	//IMU数据
	std::vector<sensor_msgs::Imu> imuDataVec;

	//是否启用外参
	bool extrinsicEnable;

	//外参
	std::array<float, 6> extrinsicParam;

	//输入读取文件路径和前面丢弃的数据包个数
	LivoxFileReader lfr(path_lvx, 2293);

	//获取数据
	lfr.getData(lidarDataVec, imuDataVec, extrinsicEnable, extrinsicParam);
	
	
	system("pause");
	return 0;
}