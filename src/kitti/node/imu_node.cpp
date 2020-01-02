#include "kitti/node/imu_node.h"




namespace kitti {
	
ImuNode::ImuNode(ros::NodeHandle &node, const NodeParameter& param)
	: Node(node, param),
	oxts_files_(param.dataset_root_path(), "/oxts", "/data/%010d", ".txt"),
	timestamps_(param.dataset_root_path(), "/oxts", "/timestamps.txt")
{
	CHECK(timestamps_.is_open()) << "Failed to open timestamp file";
	
	set_max_count(timestamps_.count());
	imu_pub_ = ros_node_.advertise<sensor_msgs::Imu>(param.topic(), 1);
}

int ImuNode::grab(int index)
{
	data_.from_txt(get_oxts_file_name_by_index(index));
	if (data_.empty())
		return -1;
	
	if (!timestamps_.get_timestamp(timestamp_, index))
		return -1;
	
	return 0;
}

int ImuNode::process()
{
	
	return 0;
}

int ImuNode::publish(ros::Time stamp)
{
	if (data_.empty())
		return -1;
	sensor_msgs::Imu msgs;
	data_.to_ros_msgs(msgs);
	fill_frame_id(msgs.header);
	fill_stamp(msgs.header, stamp);
	imu_pub_.publish(msgs);	
	return 0;
}

string ImuNode::get_oxts_file_name_by_index(int index)
{
	return oxts_files_.get_filename(index);
}

void ImuNode::fill_frame_id(std_msgs::Header &header)
{
	header.frame_id = node_param_.frame_id();
}

void ImuNode::fill_stamp(std_msgs::Header &header, ros::Time stamp)
{
	if (node_param_.original_timestamp())
		header.stamp = timestamp_.to_ros_time();
	else	
		header.stamp = stamp;
}
	
REGISTER_NODE_CLASS(Imu);	
	
	
}//namespace kitti




























