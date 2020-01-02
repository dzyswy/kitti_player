#ifndef __KITTI_IMU_NODE_H
#define __KITTI_IMU_NODE_H





#include "kitti/node/node.h"
#include "kitti/data/oxts_data.h"

namespace kitti {
	
class ImuNode : public Node
{
public:
	ImuNode(ros::NodeHandle &node, const NodeParameter& param);	
	
	virtual int grab(int index);
	virtual int process();
	virtual int publish(ros::Time stamp);
	
protected:
	string get_oxts_file_name_by_index(int index);
	void fill_frame_id(std_msgs::Header &header);
	void fill_stamp(std_msgs::Header &header, ros::Time stamp);

protected:
	FormattedFilename oxts_files_;
	TimestampFile timestamps_;
	ros::Publisher imu_pub_;
	
	 
	OxtsData data_;
	Timestamp timestamp_;
	
};
















}//namespace kitti



#endif

