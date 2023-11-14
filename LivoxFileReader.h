#ifndef LIVOXFILEREADER_DEF_H_
#define LIVOXFILEREADER_DEF_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <tuple>

#include <sensor_msgs/Imu.h>

#include"livox_def.h"

#include "CustomMsg.h"

#define kMaxPointSize 1500		//最大的点的个数
#define kDefaultFrameDurationTime 50
#define READ_BUFFER_LEN  1024 * 1024 * 32		//缓存大小
#define MAGIC_CODE       (0xac0ea767)
#define RAW_POINT_NUM     100		//一个数据包中点的个数
#define SINGLE_POINT_NUM  96		//一个数据包中点的个数
#define DUAL_POINT_NUM    48		//一个数据包中点的个数
#define TRIPLE_POINT_NUM  30		//一个数据包中点的个数
#define IMU_POINT_NUM     1			//一个数据包中点的个数
#define MAX_POINT_NUM     100		//一个数据包中点的最大个数
#define M_PI             3.14159265358979323846		//π的值

#pragma pack(1)
typedef struct {
	uint8_t signature[16];
	uint8_t version[4];
	uint32_t magic_code;
} LvxFilePublicHeader;

typedef struct {
	uint32_t frame_duration;
	uint8_t device_count;
} LvxFilePrivateHeader;

typedef struct {
	uint8_t lidar_broadcast_code[16];
	uint8_t hub_broadcast_code[16];
	uint8_t device_index;
	uint8_t device_type;
	uint8_t extrinsic_enable;
	float roll;
	float pitch;
	float yaw;
	float x;
	float y;
	float z;
} LvxDeviceInfo;

typedef struct {
	uint8_t device_index;
	uint8_t version;
	uint8_t port_id;
	uint8_t lidar_index;
	uint8_t rsvd;
	uint32_t error_code;
	uint8_t timestamp_type;
	uint8_t data_type;
	uint8_t timestamp[8];
} LvxBasePackHeader;

typedef struct {
	union
	{
		struct
		{
			uint8_t device_index;
			uint8_t version;
			uint8_t port_id;
			uint8_t lidar_index;
			uint8_t rsvd;
			uint32_t error_code;
			uint8_t timestamp_type;
			uint8_t data_type;
			uint8_t timestamp[8];
		};
		LvxBasePackHeader header;
	};
	uint8_t raw_point[kMaxPointSize];
	uint32_t pack_size;
} LvxBasePackDetail;

typedef struct {
	uint64_t current_offset;	//该文件中当前帧的绝对偏移量
	uint64_t next_offset;		//该文件中下一个帧的绝对偏移量
	uint64_t frame_index;		//当前帧索引
} FrameHeader;

#pragma pack()
typedef struct {
	union
	{
		struct
		{
			uint64_t current_offset;
			uint64_t next_offset;
			uint64_t frame_index;
		};
		FrameHeader header;
	};
	std::vector<LvxBasePackDetail> packs_vec;
} Frame;

#pragma endregion

extern std::map<DeviceType, std::string> DeviceTypeNameMap;
extern std::map<TimestampType, std::string> TimestampTypeNameMap;
extern std::map< PointDataType, std::pair<uint16_t, uint32_t> > PointBytesInfoMap;
extern std::map< PointDataType, std::string > PointTypeNameMap;

class LivoxFileReader
{
public:
	LivoxFileReader(std::string filePath, uint64_t num_throw);

	void getData(std::vector<livox_ros_driver::CustomMsg> &lidarDataVec,
				std::vector<sensor_msgs::Imu> &imuDataVec,
				bool &extrinsicEnable,
				std::array<float, 6> &extrinsicParam);

private:
	uint64_t getFileSize(const std::string file_name);  //获得输入文件的大小

	void readAllData();    //读取所有数据
	int getIMUDate();	//获取保存IMU信息的vec
	//int getLidarRate();
	int getLidarDate();		//获取保存雷达信息的vec


	void getPublicHeader();		//获得公共头文件
	void getPrivateHeader();	//获得私有头文件
	void getDeviceInfo();		//获得设备信息
	void getPointCloudData();	//获得点云数据
	void getTimeStampSecond(uint8_t *time_stamp, TimestampType time_stamp_type, 
							uint64_t &time_base, uint32_t &second, uint32_t &nsecond);
	
	uint64_t num_throw_packet_;  //前面多少个数据包丢掉不要
	std::fstream lvx_file_;  //输入流
	uint64_t file_size_;		//输入lvx文件的大小，单位是byte
	std::string file_name_;		//lvx文件路径
	LvxFilePublicHeader lvx_file_public_header_ = { 0 };   //公共文件头
	LvxFilePrivateHeader lvx_file_private_header_ = { 0 };		//私有文件头
	uint64_t cur_offset_ = 0;		//当前的二进制偏移
	std::vector<LvxDeviceInfo> device_info_vec_;		//存储设备信息的容器
	std::vector<Frame> frames_vec_;		//存储所有的点云数据
	uint64_t total_points_num_;			//总的点的个数，包含IMU
	std::map<PointDataType, std::set<int> > data_type_map_;		//根据包的数据类型查看点的个数的map
	uint32_t cur_frame_index_;			//当前帧的索引

	std::vector<livox_ros_driver::CustomMsg> lidar_data_vec_;
	std::vector<sensor_msgs::Imu> imu_data_vec_;

};

#endif

