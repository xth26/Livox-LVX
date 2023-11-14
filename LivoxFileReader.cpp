#include "LivoxFileReader.h"

#include<pcl/io/pcd_io.h>	//PCD��д����ص�ͷ�ļ�
#include<pcl/point_types.h> //pcl��֧�ֵĵ����͵�ͷ�ļ�

#pragma region Global Container

std::map<DeviceType, std::string> DeviceTypeNameMap
{
	{ kDeviceTypeHub,"LiDAR Hub"},
	{ kDeviceTypeLidarMid40,"Mid-40/Mid-100"},
	{ kDeviceTypeLidarTele,"Tele-15"},
	{ kDeviceTypeLidarHorizon,"Horizon"},
	{ kDeviceTypeLidarMid70,"Mid-70"},
	{ kDeviceTypeLidarAvia,"Avia"}
};

std::map<TimestampType, std::string> TimestampTypeNameMap
{
	// ref: https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol 
	//3.2 Time Stamp {#timestamp}
	{ kTimestampTypeNoSync, "No sync signal mode" },
	{ kTimestampTypePtp,    "1588v2.0 PTP sync mode" },
	{ kTimestampTypeRsvd,   "Reserved use" },
	{ kTimestampTypePpsGps, "pps+gps sync mode" },
	{ kTimestampTypePps,    "pps only sync mode" },
	{ kTimestampTypeUnknown,"Unknown mode" }
};

std::map< PointDataType, std::pair<uint16_t, uint32_t> > PointBytesInfoMap
{
	{ kCartesian,{ RAW_POINT_NUM, sizeof(LivoxRawPoint) } },
	{ kSpherical,{ RAW_POINT_NUM, sizeof(LivoxSpherPoint) } },
	{ kExtendCartesian,{ SINGLE_POINT_NUM, sizeof(LivoxExtendRawPoint) } },
	{ kExtendSpherical,{ SINGLE_POINT_NUM, sizeof(LivoxExtendSpherPoint) } },
	{ kDualExtendCartesian,{ DUAL_POINT_NUM, sizeof(LivoxDualExtendRawPoint) } },
	{ kDualExtendSpherical,{ DUAL_POINT_NUM, sizeof(LivoxDualExtendSpherPoint) } },
	{ kTripleExtendCartesian,{ TRIPLE_POINT_NUM, sizeof(LivoxTripleExtendRawPoint) } },
	{ kTripleExtendSpherical,{ TRIPLE_POINT_NUM, sizeof(LivoxTripleExtendSpherPoint) } },
	{ kImu,{ IMU_POINT_NUM, sizeof(LivoxImuPoint) } }
	// LivoxPoint, Standard point cloud format
};

std::map< PointDataType, std::string > PointTypeNameMap
{
	{ kCartesian,               "Cartesian coordinate point cloud" },
	{ kSpherical,               "Spherical coordinate point cloud" },
	{ kExtendCartesian,         "Extend cartesian coordinate point cloud" },
	{ kExtendSpherical,         "Extend spherical coordinate point cloud" },
	{ kDualExtendCartesian,     "Dual extend cartesian coordinate  point cloud" },
	{ kDualExtendSpherical,     "Dual extend spherical coordinate point cloud" },
	{ kTripleExtendCartesian,   "Triple extend cartesian coordinate  point cloud" },
	{ kTripleExtendSpherical,   "Triple extend spherical coordinate  point cloud" },
	{ kImu,                     "IMU data" },
	{ kMaxPointDataType,        "Max Point Data Type" }
};

uint64_t LivoxFileReader::getFileSize(const std::string file_name) {
	uint64_t begin_file, end_file;
	std::ifstream file(file_name, std::ios::in | std::ios::binary);
	begin_file = file.tellg();
	file.seekg(0, std::ios::end);
	end_file = file.tellg();
	file.close();
	return (end_file - begin_file);
}


LivoxFileReader::LivoxFileReader(std::string filePath, uint64_t num_throw)
	:cur_frame_index_(0),
	cur_offset_(0),
	total_points_num_(0)
{
	file_name_ = filePath;
	file_size_ = getFileSize(file_name_);
	num_throw_packet_ = num_throw;
}

void LivoxFileReader::getPublicHeader()
{
	int header_size = sizeof(LvxFilePublicHeader);

	std::unique_ptr<char[]> read_buffer(new char[READ_BUFFER_LEN]);
	lvx_file_.read((char *)read_buffer.get(), header_size);

	//cur_offset_ ��ʼֵΪ0
	//�����read_buffer��ȡ�������ļ�ͷ
	memcpy(&lvx_file_public_header_, read_buffer.get(), sizeof(LvxFilePublicHeader));

	//ȡ������ƫ����Ҫ���Ϲ����ļ�ͷ���ֽ�����ʣ�µľ���˽���ļ�ͷ��
	cur_offset_ += sizeof(LvxFilePublicHeader);

	//��Щ���ǹ����ļ�ͷ������
	//��������ļ�ͷ����
	std::cout << "Public Header" << std::endl;
	printf("  File Signature: %s\n", lvx_file_public_header_.signature);
	printf("  Version-A: %d\n", lvx_file_public_header_.version[0]);
	printf("  Version-B: %d\n", lvx_file_public_header_.version[1]);
	printf("  Version-C: %d\n", lvx_file_public_header_.version[2]);
	printf("  Version-D: %d\n", lvx_file_public_header_.version[3]);
	printf("  Magic Code: %x\n", lvx_file_public_header_.magic_code);
	std::cout << std::endl;
}

void LivoxFileReader::getPrivateHeader()
{
	int header_size = sizeof(LvxFilePrivateHeader);

	std::unique_ptr<char[]> read_buffer(new char[READ_BUFFER_LEN]);
	lvx_file_.read((char *)read_buffer.get(), header_size);

	//�ÿ�������ȡ��˽���ļ�ͷ������
	memcpy(&lvx_file_private_header_, read_buffer.get(), sizeof(LvxFilePrivateHeader));
	//Ȼ��ƫ��������˽���ļ�ͷ���ֽ���
	cur_offset_ += sizeof(LvxFilePrivateHeader);

	//���˽�й���ͷ�ļ�������
	std::cout << "Private Header" << std::endl;
	printf("  Per Frame Duration: %dms\n", lvx_file_private_header_.frame_duration);
	printf("  Device Count: %d\n", lvx_file_private_header_.device_count);
	std::cout << std::endl;
}

void LivoxFileReader::getDeviceInfo()
{
	//�豸����
	int device_num = lvx_file_private_header_.device_count;
	//�����豸�ĸ�������Devices Info Block�Ĵ�С������ֻ��һ���豸���������������ֻ����һ��Ԫ��
	device_info_vec_.resize(device_num);

	std::unique_ptr<char[]> read_buffer(new char[READ_BUFFER_LEN]);

	//��������ȡ�����ֽ��������ݵ�read_buffer��
	int device_info_size = device_num * sizeof(LvxDeviceInfo);

	//��fstream���ȡ���ݵ�read_buffer�������һ��device info������
	lvx_file_.read((char *)read_buffer.get(), device_info_size);

	uint64_t cur_device_info_offset = 0;

	//��������豸��device info��Ϣ
	std::cout << "Devices Info" << std::endl;

	for (int i = 0; i < device_num; ++i)
	{
		//��read_buffer����һ���豸��LvxDeviceInfo������
		memcpy(&device_info_vec_[i], read_buffer.get() + cur_device_info_offset, sizeof(LvxDeviceInfo));

		//����ƫ��������һ���豸��device info���ֽ���
		cur_offset_ += sizeof(LvxDeviceInfo);

		//��ǰƫ��������һ���豸��device info���ֽ���
		cur_device_info_offset += sizeof(LvxDeviceInfo);

		//���������device info������
		std::cout << "  LiDAR SN Code: " << device_info_vec_[i].lidar_broadcast_code << std::endl;
		std::cout << "  Hub SN Code: " << device_info_vec_[i].hub_broadcast_code << std::endl;
		std::cout << "  Device Index: " << std::to_string(device_info_vec_[i].device_index) << std::endl;
		std::cout << "  Device Type: " << DeviceTypeNameMap[DeviceType(device_info_vec_[i].device_type)].c_str() << std::endl;
		std::cout << "  Extrinsic Enable: " << (device_info_vec_[i].extrinsic_enable ? "Yes" : "No") << std::endl;
		std::cout << "  Roll: " << device_info_vec_[i].roll << std::endl;
		std::cout << "  Pitch: " << device_info_vec_[i].pitch << std::endl;
		std::cout << "  Yaw: " << device_info_vec_[i].yaw << std::endl;
		std::cout << "  X: " << device_info_vec_[i].x << std::endl;
		std::cout << "  Y: " << device_info_vec_[i].y << std::endl;
		std::cout << "  Z: " << device_info_vec_[i].z << std::endl;
		std::cout << std::endl;
	}
}

void LivoxFileReader::getPointCloudData()
{
	//ǰ���������飬�����ļ�ͷ��˽���ļ�ͷ���豸��Ϣ
	//�豸��Ϣ��Ҫ��˽���ļ�ͷ�ж�ȡ�豸������Ȼ�������ܵ��ֽ���
	//cur_offset_����ǰ����������ֽ���
	//�ȶ�ȡһ֡���ļ�ͷ
	std::cout << "Point Cloud Data" << std::endl;

	while (cur_offset_ < file_size_)
	{
		//read �� memcpy ͬ�����֣��������
		//��ǰ֡ƫ����
		uint64_t cur_frame_offset = 0;

		//���һ֡���ݵı���
		Frame cur_frame = { 0 };

		//�����read_buffer.get()��ȡ�����ݣ��������ȡ�����ˣ����ô�ͷ��ʼ��
		//������ȥȡ��һ֡ͷ�ļ����ֽڳ���
		std::unique_ptr<char[]> read_buffer(new char[READ_BUFFER_LEN]);
		lvx_file_.read((char*)read_buffer.get(), sizeof(FrameHeader));

		//��read_buffer��ȡ��һ֡��ͷ�ļ�
		memcpy(&cur_frame.header, read_buffer.get(), sizeof(FrameHeader));

		//��֡��ͷ�ļ����ֽ������뵱ǰ֡��ƫ��
		cur_frame_offset += sizeof(FrameHeader);

		//cur_offset_ ��ǰ������������ֽ���
		//current_offset �ǵ�ǰ֡�ľ���ƫ�Ƶ��ֽ�����
		//�����ƫ���Ǵ������ʼ�ĵط���ʼ���
		//�������������ȣ���ᱨ��
		//��¼������һ֡��ʼ�ĵط�
		//assert(cur_offset_ == cur_frame.current_offset);

		//read each package from package 0 - package N
		//�ӵ�һ�����ݰ��������һ�����ݰ�

		//���
		//��ǰ���ܵ�ƫ�������ϵ�ǰ֡�����ƫ��
		//С��
		//��һ֡�ľ���ƫ����������������һ֡ĩβ�ĵط�
		//��һֱ��ȡ���ݰ�
		while (cur_offset_ + cur_frame_offset < cur_frame.next_offset)
		{
			//�ȶ���һ�����ݰ�
			LvxBasePackDetail cur_pack;

			//fstream�ȼ��ϵ�ǰ֡�����ƫ�ƣ��ٴ��ж�ȡһ�����ݰ���ͷ�ļ���С
			lvx_file_.read((char*)read_buffer.get() + cur_frame_offset, sizeof(LvxBasePackHeader));

			//�ٴ�read_buffer�п���һ�����ݰ���ͷ�ļ�
			//��ʱ��read_buffer����֡��ͷ�ļ���һ�����ݰ���ͷ�ļ�
			memcpy(&cur_pack.header, read_buffer.get() + cur_frame_offset, sizeof(LvxBasePackHeader));

			//�����󣬵�ǰ֡��ƫ��Ҫ����һ�����ݰ���ͷ�ļ��ֽ�����С
			cur_frame_offset += sizeof(LvxBasePackHeader);

			//����һ���������ܵ�ǰ���ݰ�����������
			int data_type = cur_pack.data_type;

			//���������ռ���ֽ���
			uint32_t points_byte_size = 0;

			/*switch (data_type)
			{
			case PointDataType::kCartesian:
				points_byte_size = RAW_POINT_NUM * sizeof(LivoxRawPoint);
				total_points_num_ += RAW_POINT_NUM;
				break;
			case PointDataType::kSpherical:
				points_byte_size = RAW_POINT_NUM * sizeof(LivoxSpherPoint);
				total_points_num_ += RAW_POINT_NUM;
				break;
			case PointDataType::kExtendCartesian:
				points_byte_size = SINGLE_POINT_NUM * sizeof(LivoxExtendRawPoint);
				total_points_num_ += SINGLE_POINT_NUM;
				break;
			case PointDataType::kExtendSpherical:
				points_byte_size = SINGLE_POINT_NUM * sizeof(LivoxExtendSpherPoint);
				total_points_num_ += SINGLE_POINT_NUM;
				break;
			case PointDataType::kDualExtendCartesian:
				points_byte_size = DUAL_POINT_NUM * sizeof(LivoxDualExtendRawPoint);
				total_points_num_ += DUAL_POINT_NUM;
				break;
			case PointDataType::kDualExtendSpherical:
				points_byte_size = DUAL_POINT_NUM * sizeof(LivoxDualExtendSpherPoint);
				total_points_num_ += DUAL_POINT_NUM;
				break;
			case PointDataType::kImu:
				points_byte_size = IMU_POINT_NUM * sizeof(LivoxImuPoint);
				total_points_num_ += IMU_POINT_NUM;
				break;
			case PointDataType::kTripleExtendCartesian:
				points_byte_size = TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendRawPoint);
				total_points_num_ += TRIPLE_POINT_NUM;
				break;
			case PointDataType::kTripleExtendSpherical:
				points_byte_size = TRIPLE_POINT_NUM * sizeof(LivoxTripleExtendSpherPoint);
				total_points_num_ += TRIPLE_POINT_NUM;
				break;
			default:
				assert(true);
				break;
			}*/

			//first Ϊ��ĸ���
			//second Ϊһ���������ռ���ֽ���
			//����imu�Ƚ����⣬�൱��ֻ��һ����
			//PointDataType(data_type) Ϊһ��ö����������Ԫ��
			points_byte_size = PointBytesInfoMap[PointDataType(data_type)].first
				*PointBytesInfoMap[PointDataType(data_type)].second;

	/*		if (PointDataType(data_type) == kImu)
			{
				std::cout << "current frame is: " << cur_frame_index_ << std::endl;
			}*/

			//�����¼�ܵĵ�ĸ�������Ȼ��Щ��ͬʱ���������������
			total_points_num_ += PointBytesInfoMap[PointDataType(data_type)].first;

			//statistics the data type
			//�����data_type_map_��������ݣ�<PointDataType, device_index>
			//��֪������豸������ʲô��
			data_type_map_[PointDataType(data_type)].insert(cur_pack.device_index);

			//��ǰ���Ĵ�С = ����ͷ�ļ���С + ����ܵ��ֽ�����С
			cur_pack.pack_size = sizeof(LvxBasePackHeader) + points_byte_size;

			//����ƫ�ƣ�����read_buffer��������ݣ���Ȼ�ᱻ���ǵ�
			lvx_file_.read((char*)read_buffer.get() + cur_frame_offset, points_byte_size);

			//��read_buffer�п�������
			memcpy(cur_pack.raw_point, read_buffer.get() + cur_frame_offset, points_byte_size);

			//��ǰ֡��ƫ�� Ҫ���� ����ܵ��ֽ����Ĵ�С
			cur_frame_offset += points_byte_size;

			//std::vector<LvxBasePackDetail> packs_vec;
			//�������ݰ�������
			cur_frame.packs_vec.push_back(cur_pack);
			// cout << cur_pack.raw_point !!!
		}

		//��������һ֡���������ݣ�������һ֡��ƫ����
		cur_offset_ += cur_frame_offset;

		//����ۻ��ľ���ƫ�����Ƿ����һ֡�Ŀ�ʼ���
		//assert(cur_offset_ == cur_frame.next_offset);

		//vector<Frame> frames_vec_;
		//��ǰ֡�����ܵ�֡������
		frames_vec_.push_back(cur_frame);

		//��¼�ܵ�֡��
		cur_frame_index_++;
	}

	//����ܵ�֡��
	printf("  Total Frames: %d\n", cur_frame_index_);
	std::cout << std::endl;
}

void LivoxFileReader::readAllData()
{
	//�������lvx�ļ���û��ɶ���⺯��
	lvx_file_.open(file_name_, std::ios::in | std::ios::binary);

	if (!lvx_file_.is_open()) {
		std::cout << "Open lvx files failed!" << std::endl;
		return;
	}

	//��ù����ļ�ͷ����
	getPublicHeader();

	//���˽���ļ�ͷ����
	getPrivateHeader();

	//����豸��Ϣ
	getDeviceInfo();

	//���������Ϣ
	getPointCloudData();

	std::cout << "Read all data done!" << std::endl;
	std::cout << "\n";
}

//int LivoxFileReader::getLidarRate()
//{	
//	//��֮���ʱ��������λns
//	uint32_t point_interval = 4167; 
//
//	uint64_t num_packet = 0;
//
//	//��������֡
//	for (size_t i = 0; i < frames_vec_.size(); ++i)
//	{	
//		//����һ������������һ֡����״�����
//		livox_ros_driver::CustomMsg cur_curtom_msg;
//
//		cur_curtom_msg.lidar_id = 0;
//		cur_curtom_msg.rsvd = { 0,0,0 };
//
//		Frame cur_frame = frames_vec_[i];	//��ǰ֡
//		FrameHeader cur_frame_header = cur_frame.header;	//��ǰ֡��ͷ�ļ�
//		uint64_t cur_frame_idx = cur_frame_header.frame_index;		//��ǰ֡������
//		
//		//��¼�ܵĵ�ĸ���
//		uint32_t cur_point_num = 0;
//
//		//�����������ݰ�
//		for (size_t j = 0; j < cur_frame.packs_vec.size(); ++j)
//		{	
//			LvxBasePackDetail cur_pack = cur_frame.packs_vec[j];
//
//			//����һ֡�еĵ�һ�����ݰ���ʱ���
//			uint64_t stamp_full = 0;    //ʱ�����������
//			uint32_t stamp_secs = 0;	//ʱ������벿��
//			uint32_t stamp_nsecs = 0;	//ʱ�����ns����
//
//			//����һ�����飬����ֱ���ã����ú���ת��һ��
//			uint8_t *time_stamp = cur_frame.packs_vec[j].timestamp;
//			TimestampType time_stamp_type = TimestampType(cur_frame.packs_vec[j].timestamp_type);
//			getTimeStampSecond(time_stamp, time_stamp_type, stamp_full, stamp_secs, stamp_nsecs);
//
//			uint32_t packet_offset_time;
//			
//			if (j == 0)
//			{	
//				//�����ļ�ͷ����
//				std_msgs::Header msg_header;
//				msg_header.frame_id = "livox_frame";
//				msg_header.seq = cur_frame_idx;
//
//				//��һ֡�еĵ�һ�����ݰ���ʱ�����Ϊ��һ֡ͷ�ļ���ʱ���
//				msg_header.stamp.sec = stamp_secs;
//				msg_header.stamp.nsec = stamp_nsecs;
//
//				//����һ֡��ͷ�ļ�
//				cur_curtom_msg.header = msg_header;
//
//				//Ҳ��Ϊ��һ֡�Ļ���ʱ��
//				cur_curtom_msg.timebase = stamp_full;
//
//				//����ǵ�һ�����ݰ�������ƫ��ʱ��Ϊ0
//				packet_offset_time = 0;
//			}
//			else
//			{	
//				//�õ�ǰ���ݰ���ʱ�����ȥ��һ�����ݰ���ʱ���������ĵ�λ��ns
//				packet_offset_time = uint32_t(stamp_full - cur_curtom_msg.timebase);
//			}
//
//			//����һ���������ܵ�ǰ���ݰ�����������
//			int data_type = cur_pack.data_type;
//
//			//�ܵĵ�ĸ���
//			uint16_t num_points = PointBytesInfoMap[PointDataType(data_type)].first;
//
//			//����ǵ��ز�ֱ������
//			if (PointDataType(data_type) == kExtendCartesian)
//			{
//				//һ������ֽ���
//				uint32_t point_byte = PointBytesInfoMap[kExtendCartesian].second;
//
//				//��ǰ����ƫ��
//				uint64_t cur_packet_offset = 0;
//
//				uint8_t line_id = 0;
//				for (size_t k = 0; k < num_points; ++k)
//				{
//					//����һ���ṹ��������Ӱ���������ȡ������
//					LivoxExtendRawPoint raw_point;
//					memcpy(&raw_point, cur_pack.raw_point + cur_packet_offset, point_byte);
//					cur_packet_offset += sizeof(LivoxExtendRawPoint);
//
//					livox_ros_driver::CustomPoint cus_point;
//					cus_point.x = float(raw_point.x) / 1000;		//��λ�Ӻ��׵��ף�����Ҫ����1000
//					cus_point.y = float(raw_point.y) / 1000;
//					cus_point.z = float(raw_point.z) / 1000;
//					cus_point.reflectivity = raw_point.reflectivity;
//					cus_point.tag = raw_point.tag;
//
//					//һ�����ݰ����ߺţ���0��5����ȡ
//					cus_point.line = line_id % 6;
//					line_id++;
//
//					//һ�����ݰ���ƫ��ʱ�䣬����ƫ�Ƽ��ϵ��ʱ��������
//					cus_point.offset_time = packet_offset_time + k * point_interval;
//
//					//�����
//					cur_curtom_msg.points.push_back(cus_point);
//
//					//��¼��ĸ���
//					cur_point_num++;
//				}
//
//			}
//		}
//		//�������������һ֡�ĵ�������
//
//		//��¼��һ֡��ĸ���
//		cur_curtom_msg.point_num = cur_point_num;
//
//		//���뵱ǰ֡
//		lidar_data_vec_.push_back(cur_curtom_msg);
//	}
//
//	return 0;
//}

int LivoxFileReader::getLidarDate()
{
	//��֮���ʱ��������λns
	uint32_t point_interval = 4167;

	uint64_t num_packet = 0;

	uint64_t num_packet_lidar = 0;

	//����һ������������һ֡����״�����
	livox_ros_driver::CustomMsg cur_curtom_msg;

	cur_curtom_msg.lidar_id = 0;
	cur_curtom_msg.rsvd = { 0,0,0 };

	//��¼�ܵĵ�ĸ���
	uint32_t cur_point_num = 0;

	//��¼֡�ĸ���
	uint64_t seq_frame = 0;

	uint64_t last_frame_stamp = 0;

	std::vector<uint64_t> diff_stamp_com;

	uint64_t threshold_diff = 0;

	//��������֡
	for (size_t i = 0; i < frames_vec_.size(); ++i)
	{
		Frame cur_frame = frames_vec_[i];	//��ǰ֡
		FrameHeader cur_frame_header = cur_frame.header;	//��ǰ֡��ͷ�ļ�

		//�����������ݰ�
		for (size_t j = 0; j < cur_frame.packs_vec.size(); ++j)
		{
			LvxBasePackDetail cur_pack = cur_frame.packs_vec[j];

			//����һ���������ܵ�ǰ���ݰ�����������
			int data_type = cur_pack.data_type;

			//IMU���ݰ����ö�ȡ
			if (PointDataType(data_type) != kExtendCartesian)
				continue;

			//������״����ݰ�����¼һ�¸���
			if (PointDataType(data_type) == kExtendCartesian)
				num_packet++;

			//����һ֡�еĵ�һ�����ݰ���ʱ���
			uint64_t stamp_full = 0;    //ʱ�����������
			uint32_t stamp_secs = 0;	//ʱ������벿��
			uint32_t stamp_nsecs = 0;	//ʱ�����ns����

			//����һ�����飬����ֱ���ã����ú���ת��һ��
			uint8_t *time_stamp = cur_frame.packs_vec[j].timestamp;
			TimestampType time_stamp_type = TimestampType(cur_frame.packs_vec[j].timestamp_type);
			getTimeStampSecond(time_stamp, time_stamp_type, stamp_full, stamp_secs, stamp_nsecs);

			uint32_t packet_offset_time;

			/*if (stamp_full < 40549659040)*/
			if(num_packet <= num_throw_packet_)
				continue;
	
			//֡������һ����λ��5֡��ĵ�һ֡
			if ((seq_frame % 5 == 0) && (num_packet_lidar % 125 == 0))
			{
				threshold_diff = 50000000;
			}
			else
			{
				threshold_diff = 49000000;
			}

			if ((num_packet_lidar % 125 == 0) && (stamp_full - last_frame_stamp < threshold_diff))
				continue;

			if ((num_packet_lidar % 125 == 0) && (stamp_full - last_frame_stamp > threshold_diff))  //ʱ������0.05������
			{	
				uint64_t diff_stamp = stamp_full - last_frame_stamp;
				diff_stamp_com.push_back(diff_stamp);

				//�����ļ�ͷ����
				std_msgs::Header msg_header;
				msg_header.frame_id = "livox_frame";
				msg_header.seq = seq_frame;

				//֡�ĸ�����һ
				seq_frame++;

				//��һ֡�еĵ�һ�����ݰ���ʱ�����Ϊ��һ֡ͷ�ļ���ʱ���
				msg_header.stamp.sec = stamp_secs;
				msg_header.stamp.nsec = stamp_nsecs;

				//����һ֡��ͷ�ļ�
				cur_curtom_msg.header = msg_header;

				//Ҳ��Ϊ��һ֡�Ļ���ʱ��
				cur_curtom_msg.timebase = stamp_full;

				//������һ֡��ʱ���
				last_frame_stamp = stamp_full;

				//����ǵ�һ�����ݰ�������ƫ��ʱ��Ϊ0
				packet_offset_time = 0;
			}
			else
			{
				//�õ�ǰ���ݰ���ʱ�����ȥ��һ�����ݰ���ʱ���������ĵ�λ��ns
				packet_offset_time = uint32_t(stamp_full - cur_curtom_msg.timebase);
			}

			//if (j == 0)
			//{
			//	//�����ļ�ͷ����
			//	std_msgs::Header msg_header;
			//	msg_header.frame_id = "livox_frame";
			//	msg_header.seq = cur_frame_idx;

			//	//��һ֡�еĵ�һ�����ݰ���ʱ�����Ϊ��һ֡ͷ�ļ���ʱ���
			//	msg_header.stamp.sec = stamp_secs;
			//	msg_header.stamp.nsec = stamp_nsecs;

			//	//����һ֡��ͷ�ļ�
			//	cur_curtom_msg.header = msg_header;

			//	//Ҳ��Ϊ��һ֡�Ļ���ʱ��
			//	cur_curtom_msg.timebase = stamp_full;

			//	//����ǵ�һ�����ݰ�������ƫ��ʱ��Ϊ0
			//	packet_offset_time = 0;
			//}
			//else
			//{
			//	//�õ�ǰ���ݰ���ʱ�����ȥ��һ�����ݰ���ʱ���������ĵ�λ��ns
			//	packet_offset_time = uint32_t(stamp_full - cur_curtom_msg.timebase);
			//}


			//�ܵĵ�ĸ���
			uint16_t num_points = PointBytesInfoMap[PointDataType(data_type)].first;

			//����ǵ��ز�ֱ������
			if (PointDataType(data_type) == kExtendCartesian)
			{	
				num_packet_lidar++;

				//һ������ֽ���
				uint32_t point_byte = PointBytesInfoMap[kExtendCartesian].second;

				//��ǰ����ƫ��
				uint64_t cur_packet_offset = 0;

				uint8_t line_id = 0;
				for (size_t k = 0; k < num_points; ++k)
				{
					//����һ���ṹ��������Ӱ���������ȡ������
					LivoxExtendRawPoint raw_point;
					memcpy(&raw_point, cur_pack.raw_point + cur_packet_offset, point_byte);
					cur_packet_offset += sizeof(LivoxExtendRawPoint);

					livox_ros_driver::CustomPoint cus_point;
					cus_point.x = float(raw_point.x) / 1000;		//��λ�Ӻ��׵��ף�����Ҫ����1000
					cus_point.y = float(raw_point.y) / 1000;
					cus_point.z = float(raw_point.z) / 1000;
					cus_point.reflectivity = raw_point.reflectivity;
					cus_point.tag = raw_point.tag;

					//һ�����ݰ����ߺţ���0��5����ȡ
					cus_point.line = line_id % 6;
					line_id++;

					//һ�����ݰ���ƫ��ʱ�䣬����ƫ�Ƽ��ϵ��ʱ��������
					cus_point.offset_time = packet_offset_time + k * point_interval;

					//�����
					cur_curtom_msg.points.push_back(cus_point);

					//��¼��ĸ���
					cur_point_num++;
				}

			}

			//����湻��125�������ͱ�������������մ�ŵ������
			if (num_packet_lidar % 125 == 0)
			{
				//��¼��һ֡��ĸ���
				cur_curtom_msg.point_num = cur_point_num;

				//���뵱ǰ֡
				lidar_data_vec_.push_back(cur_curtom_msg);

				//��ձ����ĵ���
				cur_curtom_msg.points.clear();

				//��¼һ֡��������ı�������
				cur_point_num = 0;
			}

			if (i == frames_vec_.size() - 1 && j == cur_frame.packs_vec.size() - 1)
			{
				//��¼��һ֡��ĸ���
				cur_curtom_msg.point_num = cur_point_num;

				//���뵱ǰ֡
				lidar_data_vec_.push_back(cur_curtom_msg);

				//��ձ����ĵ���
				cur_curtom_msg.points.clear();

				//��¼һ֡��������ı�������
				cur_point_num = 0;
			}

			//if (lidar_data_vec_.size() == 1000)
			//	std::cout << "pause! " << std::endl;

		}
		//�������������һ֡�ĵ�������
	}

	return 0;
}

int LivoxFileReader::getIMUDate()
{	
	//��¼����imu�ļ������
	uint32_t seq_imu = 0;

	//��������֡
	for (size_t i = 0; i < frames_vec_.size(); ++i)
	{
		Frame cur_frame = frames_vec_[i];	//��ǰ֡
		FrameHeader cur_frame_header = cur_frame.header;	//��ǰ֡��ͷ�ļ�
		uint64_t cur_frame_idx = cur_frame_header.frame_index;		//��ǰ֡������

		//�����������ݰ�
		for (size_t j = 0; j < cur_frame.packs_vec.size(); ++j)
		{	
			LvxBasePackDetail cur_pack = cur_frame.packs_vec[j];

			//����һ���������ܵ�ǰ���ݰ�����������
			int data_type = cur_pack.data_type;

			//�ܵĵ�ĸ���
			uint16_t num_points = PointBytesInfoMap[PointDataType(data_type)].first;
	
			//�����IMU��������
			if (PointDataType(data_type) == kImu)
			{	
				//һ������ֽ���
				uint32_t point_byte = PointBytesInfoMap[kImu].second;

				//����һ���ṹ�����������������ȡ������
				LivoxImuPoint raw_point;
				memcpy(&raw_point, cur_pack.raw_point, point_byte);

				//����һ������������һ��IMU����
				sensor_msgs::Imu cur_imu_msg;

				//Э����ø�ֵ��Ĭ��Ϊ0
				////����Э����
				cur_imu_msg.angular_velocity_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };		//���ٶ�Э����
				cur_imu_msg.linear_acceleration_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };	//���ٶ�Э����
				cur_imu_msg.orientation_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };		//����Э����
				
				cur_imu_msg.orientation.x = double(0);
				cur_imu_msg.orientation.y = double(0);
				cur_imu_msg.orientation.z = double(0);
				cur_imu_msg.orientation.w = double(0);

				cur_imu_msg.angular_velocity.x = raw_point.acc_x;
				cur_imu_msg.angular_velocity.y = raw_point.acc_y;
				cur_imu_msg.angular_velocity.z = raw_point.acc_z;
			
				cur_imu_msg.linear_acceleration.x = raw_point.gyro_x;
				cur_imu_msg.linear_acceleration.y = raw_point.gyro_y;
				cur_imu_msg.linear_acceleration.z = raw_point.gyro_z;

				//����һ֡�еĵ�һ�����ݰ���ʱ���
				uint64_t time_base = 0;    //ʱ�����������
				uint32_t stamp_secs = 0;	//ʱ������벿��
				uint32_t stamp_nsecs = 0;	//ʱ�����ns����

				uint8_t *time_stamp = cur_pack.timestamp;
				TimestampType time_stamp_type = TimestampType(cur_pack.timestamp_type);
				getTimeStampSecond(time_stamp, time_stamp_type, time_base, stamp_secs, stamp_nsecs);

				//�����ļ�ͷ����
				std_msgs::Header msg_header;
				msg_header.frame_id = "livox_frame";
				msg_header.stamp.sec = stamp_secs;
				msg_header.stamp.nsec = stamp_nsecs;
				msg_header.seq = seq_imu;
				seq_imu++;

				//����ͷ�ļ�
				cur_imu_msg.header = msg_header;

				imu_data_vec_.push_back(cur_imu_msg);

			}		
		}
		//�������������һ֡�ĵ�������

	}

	return 0;
}

//�������Ϊ��λ��ʱ�����С���������9λ
void LivoxFileReader::getTimeStampSecond(uint8_t *time_stamp, TimestampType time_stamp_type, uint64_t &time_base,uint32_t &second, uint32_t &nsecond)
{

	if (time_stamp_type == kTimestampTypeNoSync ||
		time_stamp_type == kTimestampTypePtp)
	{
		// data type: uint64_t, unit: second.
		uint64_t time_;
		memcpy(&time_, time_stamp, sizeof(uint64_t));

		time_base = time_;

		second = time_ / (1.0e9);

		nsecond = time_ - second * (1.0e9);
	}
	else
	{
		std::cout << "Time stamp type is not right!" << std::endl;
	}
}

void LivoxFileReader::getData(std::vector<livox_ros_driver::CustomMsg> &lidarDataVec,
							  std::vector<sensor_msgs::Imu> &imuDataVec,
							  bool &extrinsicEnable,
							  std::array<float,6> &extrinsicParam)
{	
	//���ļ��ж�ȡ��������
	readAllData();

	//��ȡ�ⲿ�������Լ��Ƿ������ⲿ����
	LvxDeviceInfo device_info_0 = device_info_vec_[0];
	
	if (device_info_0.extrinsic_enable == 0)
	{
		extrinsicEnable = false;   //������
		extrinsicParam = { 0,0,0,0,0,0};
	}
	else
	{
		extrinsicEnable = true;	  //����
		extrinsicParam = {
		device_info_0.roll,
		device_info_0.pitch,
		device_info_0.yaw,
		device_info_0.x,
		device_info_0.y,
		device_info_0.z
		};
	}

	//���IMU����
	if (!getIMUDate())
	{	
		imuDataVec = imu_data_vec_;
		std::cout << "Get IMU rate done!" << std::endl;
	}

	//����״�����
	if (!getLidarDate())
	{	
		lidarDataVec = lidar_data_vec_;
		std::cout << "Get Lidar rate done!" << std::endl;
	}
}