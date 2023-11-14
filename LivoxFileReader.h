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

#define kMaxPointSize 1500		//���ĵ�ĸ���
#define kDefaultFrameDurationTime 50
#define READ_BUFFER_LEN  1024 * 1024 * 32		//�����С
#define MAGIC_CODE       (0xac0ea767)
#define RAW_POINT_NUM     100		//һ�����ݰ��е�ĸ���
#define SINGLE_POINT_NUM  96		//һ�����ݰ��е�ĸ���
#define DUAL_POINT_NUM    48		//һ�����ݰ��е�ĸ���
#define TRIPLE_POINT_NUM  30		//һ�����ݰ��е�ĸ���
#define IMU_POINT_NUM     1			//һ�����ݰ��е�ĸ���
#define MAX_POINT_NUM     100		//һ�����ݰ��е��������
#define M_PI             3.14159265358979323846		//�е�ֵ

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
	uint64_t current_offset;	//���ļ��е�ǰ֡�ľ���ƫ����
	uint64_t next_offset;		//���ļ�����һ��֡�ľ���ƫ����
	uint64_t frame_index;		//��ǰ֡����
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
	uint64_t getFileSize(const std::string file_name);  //��������ļ��Ĵ�С

	void readAllData();    //��ȡ��������
	int getIMUDate();	//��ȡ����IMU��Ϣ��vec
	//int getLidarRate();
	int getLidarDate();		//��ȡ�����״���Ϣ��vec


	void getPublicHeader();		//��ù���ͷ�ļ�
	void getPrivateHeader();	//���˽��ͷ�ļ�
	void getDeviceInfo();		//����豸��Ϣ
	void getPointCloudData();	//��õ�������
	void getTimeStampSecond(uint8_t *time_stamp, TimestampType time_stamp_type, 
							uint64_t &time_base, uint32_t &second, uint32_t &nsecond);
	
	uint64_t num_throw_packet_;  //ǰ����ٸ����ݰ�������Ҫ
	std::fstream lvx_file_;  //������
	uint64_t file_size_;		//����lvx�ļ��Ĵ�С����λ��byte
	std::string file_name_;		//lvx�ļ�·��
	LvxFilePublicHeader lvx_file_public_header_ = { 0 };   //�����ļ�ͷ
	LvxFilePrivateHeader lvx_file_private_header_ = { 0 };		//˽���ļ�ͷ
	uint64_t cur_offset_ = 0;		//��ǰ�Ķ�����ƫ��
	std::vector<LvxDeviceInfo> device_info_vec_;		//�洢�豸��Ϣ������
	std::vector<Frame> frames_vec_;		//�洢���еĵ�������
	uint64_t total_points_num_;			//�ܵĵ�ĸ���������IMU
	std::map<PointDataType, std::set<int> > data_type_map_;		//���ݰ����������Ͳ鿴��ĸ�����map
	uint32_t cur_frame_index_;			//��ǰ֡������

	std::vector<livox_ros_driver::CustomMsg> lidar_data_vec_;
	std::vector<sensor_msgs::Imu> imu_data_vec_;

};

#endif

