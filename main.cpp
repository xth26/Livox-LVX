#include<iostream>
#include<fstream>  //��ȡ����ͷ�ļ�

#include <cstdio>   //��ȡ�ļ���СҪ��
#include <string>
#include <array> 

#include<pcl/io/pcd_io.h>	//PCD��д����ص�ͷ�ļ�
#include<pcl/point_types.h> //pcl��֧�ֵĵ����͵�ͷ�ļ�

#include"LivoxFileReader.h"

int main()
{	
	//lvx�ļ�·��
	std::string path_lvx = "D:\\LAS_Files\\lvx_files\\2022_10_30.lvx";
	
	//��������
	std::vector<livox_ros_driver::CustomMsg> lidarDataVec;

	//IMU����
	std::vector<sensor_msgs::Imu> imuDataVec;

	//�Ƿ��������
	bool extrinsicEnable;

	//���
	std::array<float, 6> extrinsicParam;

	//�����ȡ�ļ�·����ǰ�涪�������ݰ�����
	LivoxFileReader lfr(path_lvx, 2293);

	//��ȡ����
	lfr.getData(lidarDataVec, imuDataVec, extrinsicEnable, extrinsicParam);
	
	
	system("pause");
	return 0;
}