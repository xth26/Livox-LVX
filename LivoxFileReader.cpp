#include "LivoxFileReader.h"

#include<pcl/io/pcd_io.h>	//PCD读写类相关的头文件
#include<pcl/point_types.h> //pcl中支持的点类型的头文件

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

	//cur_offset_ 初始值为0
	//这里从read_buffer中取出公共文件头
	memcpy(&lvx_file_public_header_, read_buffer.get(), sizeof(LvxFilePublicHeader));

	//取出来后，偏移量要加上公共文件头的字节数，剩下的就是私有文件头了
	cur_offset_ += sizeof(LvxFilePublicHeader);

	//这些都是公共文件头的内容
	//输出公共文件头内容
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

	//用拷贝函数取出私有文件头的内容
	memcpy(&lvx_file_private_header_, read_buffer.get(), sizeof(LvxFilePrivateHeader));
	//然后偏移量加上私有文件头的字节数
	cur_offset_ += sizeof(LvxFilePrivateHeader);

	//输出私有公共头文件的内容
	std::cout << "Private Header" << std::endl;
	printf("  Per Frame Duration: %dms\n", lvx_file_private_header_.frame_duration);
	printf("  Device Count: %d\n", lvx_file_private_header_.device_count);
	std::cout << std::endl;
}

void LivoxFileReader::getDeviceInfo()
{
	//设备个数
	int device_num = lvx_file_private_header_.device_count;
	//根据设备的个数决定Devices Info Block的大小，这里只有一个设备，所以这个容器里只放了一个元素
	device_info_vec_.resize(device_num);

	std::unique_ptr<char[]> read_buffer(new char[READ_BUFFER_LEN]);

	//这里计算读取多少字节数的数据到read_buffer里
	int device_info_size = device_num * sizeof(LvxDeviceInfo);

	//从fstream里读取数据到read_buffer里，数据是一个device info的内容
	lvx_file_.read((char *)read_buffer.get(), device_info_size);

	uint64_t cur_device_info_offset = 0;

	//依次输出设备的device info信息
	std::cout << "Devices Info" << std::endl;

	for (int i = 0; i < device_num; ++i)
	{
		//从read_buffer里拷贝一个设备的LvxDeviceInfo的内容
		memcpy(&device_info_vec_[i], read_buffer.get() + cur_device_info_offset, sizeof(LvxDeviceInfo));

		//绝对偏移量加上一个设备的device info的字节数
		cur_offset_ += sizeof(LvxDeviceInfo);

		//当前偏移量加上一个设备的device info的字节数
		cur_device_info_offset += sizeof(LvxDeviceInfo);

		//下面是输出device info的内容
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
	//前面有三个块，公共文件头，私有文件头和设备信息
	//设备信息需要从私有文件头中读取设备个数，然后计算出总的字节数
	//cur_offset_包括前面三个块的字节数
	//先读取一帧的文件头
	std::cout << "Point Cloud Data" << std::endl;

	while (cur_offset_ < file_size_)
	{
		//read 和 memcpy 同步出现，避免出错！
		//当前帧偏移量
		uint64_t cur_frame_offset = 0;

		//存放一帧内容的变量
		Frame cur_frame = { 0 };

		//这里的read_buffer.get()会取出数据，后面跟着取就行了，不用从头开始算
		//这里先去取出一帧头文件的字节长度
		std::unique_ptr<char[]> read_buffer(new char[READ_BUFFER_LEN]);
		lvx_file_.read((char*)read_buffer.get(), sizeof(FrameHeader));

		//从read_buffer中取出一帧的头文件
		memcpy(&cur_frame.header, read_buffer.get(), sizeof(FrameHeader));

		//把帧的头文件的字节数加入当前帧的偏移
		cur_frame_offset += sizeof(FrameHeader);

		//cur_offset_ 是前面三个块的总字节数
		//current_offset 是当前帧的绝对偏移的字节数，
		//这里的偏移是从数据最开始的地方开始算的
		//如果两个数不相等，则会报错
		//记录的是这一帧开始的地方
		//assert(cur_offset_ == cur_frame.current_offset);

		//read each package from package 0 - package N
		//从第一个数据包读到最后一个数据包

		//如果
		//当前的总的偏移量加上当前帧的相对偏移
		//小于
		//下一帧的绝对偏移量，这个相对于这一帧末尾的地方
		//则一直读取数据包
		while (cur_offset_ + cur_frame_offset < cur_frame.next_offset)
		{
			//先定义一个数据包
			LvxBasePackDetail cur_pack;

			//fstream先加上当前帧的相对偏移，再从中读取一个数据包的头文件大小
			lvx_file_.read((char*)read_buffer.get() + cur_frame_offset, sizeof(LvxBasePackHeader));

			//再从read_buffer中拷贝一个数据包的头文件
			//这时的read_buffer包含帧的头文件和一个数据包的头文件
			memcpy(&cur_pack.header, read_buffer.get() + cur_frame_offset, sizeof(LvxBasePackHeader));

			//拷贝后，当前帧的偏移要加上一个数据包的头文件字节数大小
			cur_frame_offset += sizeof(LvxBasePackHeader);

			//定义一个变量接受当前数据包的数据类型
			int data_type = cur_pack.data_type;

			//点的数据所占的字节数
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

			//first 为点的个数
			//second 为一个点的数据占的字节数
			//这里imu比较特殊，相当于只有一个点
			//PointDataType(data_type) 为一个枚举类型里面元素
			points_byte_size = PointBytesInfoMap[PointDataType(data_type)].first
				*PointBytesInfoMap[PointDataType(data_type)].second;

	/*		if (PointDataType(data_type) == kImu)
			{
				std::cout << "current frame is: " << cur_frame_index_ << std::endl;
			}*/

			//这里记录总的点的个数，虽然有些点同时包含两个点的数据
			total_points_num_ += PointBytesInfoMap[PointDataType(data_type)].first;

			//statistics the data type
			//这里给data_type_map_添加了数据，<PointDataType, device_index>
			//不知道添加设备索引有什么用
			data_type_map_[PointDataType(data_type)].insert(cur_pack.device_index);

			//当前包的大小 = 包的头文件大小 + 点的总的字节数大小
			cur_pack.pack_size = sizeof(LvxBasePackHeader) + points_byte_size;

			//加上偏移，再往read_buffer里存入数据，不然会被覆盖掉
			lvx_file_.read((char*)read_buffer.get() + cur_frame_offset, points_byte_size);

			//从read_buffer中拷贝数据
			memcpy(cur_pack.raw_point, read_buffer.get() + cur_frame_offset, points_byte_size);

			//当前帧的偏移 要加上 点的总的字节数的大小
			cur_frame_offset += points_byte_size;

			//std::vector<LvxBasePackDetail> packs_vec;
			//存入数据包容器里
			cur_frame.packs_vec.push_back(cur_pack);
			// cout << cur_pack.raw_point !!!
		}

		//到这里获得一帧的所有数据，加上这一帧的偏移量
		cur_offset_ += cur_frame_offset;

		//检查累积的绝对偏移量是否和下一帧的开始相等
		//assert(cur_offset_ == cur_frame.next_offset);

		//vector<Frame> frames_vec_;
		//当前帧存入总的帧容器里
		frames_vec_.push_back(cur_frame);

		//记录总的帧数
		cur_frame_index_++;
	}

	//输出总的帧数
	printf("  Total Frames: %d\n", cur_frame_index_);
	std::cout << std::endl;
}

void LivoxFileReader::readAllData()
{
	//从这里打开lvx文件，没有啥特殊函数
	lvx_file_.open(file_name_, std::ios::in | std::ios::binary);

	if (!lvx_file_.is_open()) {
		std::cout << "Open lvx files failed!" << std::endl;
		return;
	}

	//获得公共文件头数据
	getPublicHeader();

	//获得私有文件头数据
	getPrivateHeader();

	//获得设备信息
	getDeviceInfo();

	//获得数据信息
	getPointCloudData();

	std::cout << "Read all data done!" << std::endl;
	std::cout << "\n";
}

//int LivoxFileReader::getLidarRate()
//{	
//	//点之间的时间间隔，单位ns
//	uint32_t point_interval = 4167; 
//
//	uint64_t num_packet = 0;
//
//	//遍历所有帧
//	for (size_t i = 0; i < frames_vec_.size(); ++i)
//	{	
//		//定义一个变量，接受一帧里的雷达数据
//		livox_ros_driver::CustomMsg cur_curtom_msg;
//
//		cur_curtom_msg.lidar_id = 0;
//		cur_curtom_msg.rsvd = { 0,0,0 };
//
//		Frame cur_frame = frames_vec_[i];	//当前帧
//		FrameHeader cur_frame_header = cur_frame.header;	//当前帧的头文件
//		uint64_t cur_frame_idx = cur_frame_header.frame_index;		//当前帧的索引
//		
//		//记录总的点的个数
//		uint32_t cur_point_num = 0;
//
//		//遍历所有数据包
//		for (size_t j = 0; j < cur_frame.packs_vec.size(); ++j)
//		{	
//			LvxBasePackDetail cur_pack = cur_frame.packs_vec[j];
//
//			//接受一帧中的第一个数据包的时间戳
//			uint64_t stamp_full = 0;    //时间戳完整部分
//			uint32_t stamp_secs = 0;	//时间戳的秒部分
//			uint32_t stamp_nsecs = 0;	//时间戳的ns部分
//
//			//这是一个数组，不能直接用，得用函数转换一下
//			uint8_t *time_stamp = cur_frame.packs_vec[j].timestamp;
//			TimestampType time_stamp_type = TimestampType(cur_frame.packs_vec[j].timestamp_type);
//			getTimeStampSecond(time_stamp, time_stamp_type, stamp_full, stamp_secs, stamp_nsecs);
//
//			uint32_t packet_offset_time;
//			
//			if (j == 0)
//			{	
//				//这是文件头变量
//				std_msgs::Header msg_header;
//				msg_header.frame_id = "livox_frame";
//				msg_header.seq = cur_frame_idx;
//
//				//把一帧中的第一个数据包的时间戳作为这一帧头文件的时间戳
//				msg_header.stamp.sec = stamp_secs;
//				msg_header.stamp.nsec = stamp_nsecs;
//
//				//放入一帧的头文件
//				cur_curtom_msg.header = msg_header;
//
//				//也作为这一帧的基础时间
//				cur_curtom_msg.timebase = stamp_full;
//
//				//如果是第一个数据包，包的偏移时间为0
//				packet_offset_time = 0;
//			}
//			else
//			{	
//				//用当前数据包的时间戳减去第一个数据包的时间戳，这里的单位是ns
//				packet_offset_time = uint32_t(stamp_full - cur_curtom_msg.timebase);
//			}
//
//			//定义一个变量接受当前数据包的数据类型
//			int data_type = cur_pack.data_type;
//
//			//总的点的个数
//			uint16_t num_points = PointBytesInfoMap[PointDataType(data_type)].first;
//
//			//如果是单回波直角坐标
//			if (PointDataType(data_type) == kExtendCartesian)
//			{
//				//一个点的字节数
//				uint32_t point_byte = PointBytesInfoMap[kExtendCartesian].second;
//
//				//当前包的偏移
//				uint64_t cur_packet_offset = 0;
//
//				uint8_t line_id = 0;
//				for (size_t k = 0; k < num_points; ++k)
//				{
//					//定义一个结构体变量，从包的数据流取出数据
//					LivoxExtendRawPoint raw_point;
//					memcpy(&raw_point, cur_pack.raw_point + cur_packet_offset, point_byte);
//					cur_packet_offset += sizeof(LivoxExtendRawPoint);
//
//					livox_ros_driver::CustomPoint cus_point;
//					cus_point.x = float(raw_point.x) / 1000;		//单位从毫米到米，所以要除以1000
//					cus_point.y = float(raw_point.y) / 1000;
//					cus_point.z = float(raw_point.z) / 1000;
//					cus_point.reflectivity = raw_point.reflectivity;
//					cus_point.tag = raw_point.tag;
//
//					//一个数据包的线号，从0到5挨着取
//					cus_point.line = line_id % 6;
//					line_id++;
//
//					//一个数据包的偏移时间，包的偏移加上点的时间间隔即可
//					cus_point.offset_time = packet_offset_time + k * point_interval;
//
//					//存入点
//					cur_curtom_msg.points.push_back(cus_point);
//
//					//记录点的个数
//					cur_point_num++;
//				}
//
//			}
//		}
//		//遍历结束，获得一帧的点云数据
//
//		//记录这一帧点的个数
//		cur_curtom_msg.point_num = cur_point_num;
//
//		//存入当前帧
//		lidar_data_vec_.push_back(cur_curtom_msg);
//	}
//
//	return 0;
//}

int LivoxFileReader::getLidarDate()
{
	//点之间的时间间隔，单位ns
	uint32_t point_interval = 4167;

	uint64_t num_packet = 0;

	uint64_t num_packet_lidar = 0;

	//定义一个变量，接受一帧里的雷达数据
	livox_ros_driver::CustomMsg cur_curtom_msg;

	cur_curtom_msg.lidar_id = 0;
	cur_curtom_msg.rsvd = { 0,0,0 };

	//记录总的点的个数
	uint32_t cur_point_num = 0;

	//记录帧的个数
	uint64_t seq_frame = 0;

	uint64_t last_frame_stamp = 0;

	std::vector<uint64_t> diff_stamp_com;

	uint64_t threshold_diff = 0;

	//遍历所有帧
	for (size_t i = 0; i < frames_vec_.size(); ++i)
	{
		Frame cur_frame = frames_vec_[i];	//当前帧
		FrameHeader cur_frame_header = cur_frame.header;	//当前帧的头文件

		//遍历所有数据包
		for (size_t j = 0; j < cur_frame.packs_vec.size(); ++j)
		{
			LvxBasePackDetail cur_pack = cur_frame.packs_vec[j];

			//定义一个变量接受当前数据包的数据类型
			int data_type = cur_pack.data_type;

			//IMU数据包不用读取
			if (PointDataType(data_type) != kExtendCartesian)
				continue;

			//如果是雷达数据包，记录一下个数
			if (PointDataType(data_type) == kExtendCartesian)
				num_packet++;

			//接受一帧中的第一个数据包的时间戳
			uint64_t stamp_full = 0;    //时间戳完整部分
			uint32_t stamp_secs = 0;	//时间戳的秒部分
			uint32_t stamp_nsecs = 0;	//时间戳的ns部分

			//这是一个数组，不能直接用，得用函数转换一下
			uint8_t *time_stamp = cur_frame.packs_vec[j].timestamp;
			TimestampType time_stamp_type = TimestampType(cur_frame.packs_vec[j].timestamp_type);
			getTimeStampSecond(time_stamp, time_stamp_type, stamp_full, stamp_secs, stamp_nsecs);

			uint32_t packet_offset_time;

			/*if (stamp_full < 40549659040)*/
			if(num_packet <= num_throw_packet_)
				continue;
	
			//帧数大于一，且位于5帧里的第一帧
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

			if ((num_packet_lidar % 125 == 0) && (stamp_full - last_frame_stamp > threshold_diff))  //时间戳相差0.05秒以上
			{	
				uint64_t diff_stamp = stamp_full - last_frame_stamp;
				diff_stamp_com.push_back(diff_stamp);

				//这是文件头变量
				std_msgs::Header msg_header;
				msg_header.frame_id = "livox_frame";
				msg_header.seq = seq_frame;

				//帧的个数加一
				seq_frame++;

				//把一帧中的第一个数据包的时间戳作为这一帧头文件的时间戳
				msg_header.stamp.sec = stamp_secs;
				msg_header.stamp.nsec = stamp_nsecs;

				//放入一帧的头文件
				cur_curtom_msg.header = msg_header;

				//也作为这一帧的基础时间
				cur_curtom_msg.timebase = stamp_full;

				//存入上一帧的时间戳
				last_frame_stamp = stamp_full;

				//如果是第一个数据包，包的偏移时间为0
				packet_offset_time = 0;
			}
			else
			{
				//用当前数据包的时间戳减去第一个数据包的时间戳，这里的单位是ns
				packet_offset_time = uint32_t(stamp_full - cur_curtom_msg.timebase);
			}

			//if (j == 0)
			//{
			//	//这是文件头变量
			//	std_msgs::Header msg_header;
			//	msg_header.frame_id = "livox_frame";
			//	msg_header.seq = cur_frame_idx;

			//	//把一帧中的第一个数据包的时间戳作为这一帧头文件的时间戳
			//	msg_header.stamp.sec = stamp_secs;
			//	msg_header.stamp.nsec = stamp_nsecs;

			//	//放入一帧的头文件
			//	cur_curtom_msg.header = msg_header;

			//	//也作为这一帧的基础时间
			//	cur_curtom_msg.timebase = stamp_full;

			//	//如果是第一个数据包，包的偏移时间为0
			//	packet_offset_time = 0;
			//}
			//else
			//{
			//	//用当前数据包的时间戳减去第一个数据包的时间戳，这里的单位是ns
			//	packet_offset_time = uint32_t(stamp_full - cur_curtom_msg.timebase);
			//}


			//总的点的个数
			uint16_t num_points = PointBytesInfoMap[PointDataType(data_type)].first;

			//如果是单回波直角坐标
			if (PointDataType(data_type) == kExtendCartesian)
			{	
				num_packet_lidar++;

				//一个点的字节数
				uint32_t point_byte = PointBytesInfoMap[kExtendCartesian].second;

				//当前包的偏移
				uint64_t cur_packet_offset = 0;

				uint8_t line_id = 0;
				for (size_t k = 0; k < num_points; ++k)
				{
					//定义一个结构体变量，从包的数据流取出数据
					LivoxExtendRawPoint raw_point;
					memcpy(&raw_point, cur_pack.raw_point + cur_packet_offset, point_byte);
					cur_packet_offset += sizeof(LivoxExtendRawPoint);

					livox_ros_driver::CustomPoint cus_point;
					cus_point.x = float(raw_point.x) / 1000;		//单位从毫米到米，所以要除以1000
					cus_point.y = float(raw_point.y) / 1000;
					cus_point.z = float(raw_point.z) / 1000;
					cus_point.reflectivity = raw_point.reflectivity;
					cus_point.tag = raw_point.tag;

					//一个数据包的线号，从0到5挨着取
					cus_point.line = line_id % 6;
					line_id++;

					//一个数据包的偏移时间，包的偏移加上点的时间间隔即可
					cus_point.offset_time = packet_offset_time + k * point_interval;

					//存入点
					cur_curtom_msg.points.push_back(cus_point);

					//记录点的个数
					cur_point_num++;
				}

			}

			//如果存够了125个包，就保存起来，并清空存放点的向量
			if (num_packet_lidar % 125 == 0)
			{
				//记录这一帧点的个数
				cur_curtom_msg.point_num = cur_point_num;

				//存入当前帧
				lidar_data_vec_.push_back(cur_curtom_msg);

				//清空保存点的点云
				cur_curtom_msg.points.clear();

				//记录一帧点的总数的变量清零
				cur_point_num = 0;
			}

			if (i == frames_vec_.size() - 1 && j == cur_frame.packs_vec.size() - 1)
			{
				//记录这一帧点的个数
				cur_curtom_msg.point_num = cur_point_num;

				//存入当前帧
				lidar_data_vec_.push_back(cur_curtom_msg);

				//清空保存点的点云
				cur_curtom_msg.points.clear();

				//记录一帧点的总数的变量清零
				cur_point_num = 0;
			}

			//if (lidar_data_vec_.size() == 1000)
			//	std::cout << "pause! " << std::endl;

		}
		//遍历结束，获得一帧的点云数据
	}

	return 0;
}

int LivoxFileReader::getIMUDate()
{	
	//记录所有imu文件的序号
	uint32_t seq_imu = 0;

	//遍历所有帧
	for (size_t i = 0; i < frames_vec_.size(); ++i)
	{
		Frame cur_frame = frames_vec_[i];	//当前帧
		FrameHeader cur_frame_header = cur_frame.header;	//当前帧的头文件
		uint64_t cur_frame_idx = cur_frame_header.frame_index;		//当前帧的索引

		//遍历所有数据包
		for (size_t j = 0; j < cur_frame.packs_vec.size(); ++j)
		{	
			LvxBasePackDetail cur_pack = cur_frame.packs_vec[j];

			//定义一个变量接受当前数据包的数据类型
			int data_type = cur_pack.data_type;

			//总的点的个数
			uint16_t num_points = PointBytesInfoMap[PointDataType(data_type)].first;
	
			//如果是IMU数据类型
			if (PointDataType(data_type) == kImu)
			{	
				//一个点的字节数
				uint32_t point_byte = PointBytesInfoMap[kImu].second;

				//定义一个结构体变量，从数据流中取出数据
				LivoxImuPoint raw_point;
				memcpy(&raw_point, cur_pack.raw_point, point_byte);

				//定义一个变量，接受一个IMU数据
				sensor_msgs::Imu cur_imu_msg;

				//协方差不用赋值，默认为0
				////存入协方差
				cur_imu_msg.angular_velocity_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };		//角速度协方差
				cur_imu_msg.linear_acceleration_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };	//线速度协方差
				cur_imu_msg.orientation_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };		//方向协方差
				
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

				//接受一帧中的第一个数据包的时间戳
				uint64_t time_base = 0;    //时间戳完整部分
				uint32_t stamp_secs = 0;	//时间戳的秒部分
				uint32_t stamp_nsecs = 0;	//时间戳的ns部分

				uint8_t *time_stamp = cur_pack.timestamp;
				TimestampType time_stamp_type = TimestampType(cur_pack.timestamp_type);
				getTimeStampSecond(time_stamp, time_stamp_type, time_base, stamp_secs, stamp_nsecs);

				//这是文件头变量
				std_msgs::Header msg_header;
				msg_header.frame_id = "livox_frame";
				msg_header.stamp.sec = stamp_secs;
				msg_header.stamp.nsec = stamp_nsecs;
				msg_header.seq = seq_imu;
				seq_imu++;

				//存入头文件
				cur_imu_msg.header = msg_header;

				imu_data_vec_.push_back(cur_imu_msg);

			}		
		}
		//遍历结束，获得一帧的点云数据

	}

	return 0;
}

//获得以秒为单位的时间戳，小数点后保留了9位
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
	//从文件中读取所有数据
	readAllData();

	//读取外部参数，以及是否启用外部参数
	LvxDeviceInfo device_info_0 = device_info_vec_[0];
	
	if (device_info_0.extrinsic_enable == 0)
	{
		extrinsicEnable = false;   //不启用
		extrinsicParam = { 0,0,0,0,0,0};
	}
	else
	{
		extrinsicEnable = true;	  //启用
		extrinsicParam = {
		device_info_0.roll,
		device_info_0.pitch,
		device_info_0.yaw,
		device_info_0.x,
		device_info_0.y,
		device_info_0.z
		};
	}

	//获得IMU数据
	if (!getIMUDate())
	{	
		imuDataVec = imu_data_vec_;
		std::cout << "Get IMU rate done!" << std::endl;
	}

	//获得雷达数据
	if (!getLidarDate())
	{	
		lidarDataVec = lidar_data_vec_;
		std::cout << "Get Lidar rate done!" << std::endl;
	}
}