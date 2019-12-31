#include "kitti/node/velodyne_node.h"





namespace kitti {
VelodyneNode::VelodyneNode(ros::NodeHandle &node, const NodeParameter& param)
	: Node(node, param),
	cloud_bins_(param.dataset_root_path(), "/velodyne_points/data/%010d", ".bin"),
	timestamps_(param.dataset_root_path(), "/velodyne_points/timestamps.txt")
	
{
	CHECK(timestamps_.is_open()) << "Failed to open timestamp file";
	
	set_max_count(timestamps_.count());
	cloud_pub_ = ros_node_.advertise<sensor_msgs::PointCloud2> (param.topic(), 1);
}	

int VelodyneNode::grab(int index)
{
	points_.from_bin(get_cloud_bin_name_by_index(index));
	if (points_.empty())
		return -1;
	
	if (!timestamps_.get_timestamp(timestamp_, index))
		return -1;
	
	return 0;
}

int VelodyneNode::process()
{
	
	return 0;
}

int VelodyneNode::publish(ros::Time stamp)
{
	if (points_.empty())
		return -1;
	
	sensor_msgs::PointCloud2 cloud_msgs;
	points_.to_ros_msgs(cloud_msgs);
	fill_frame_id(cloud_msgs.header);
	fill_stamp(cloud_msgs.header, stamp);
	cloud_pub_.publish(cloud_msgs);	
	return 0;
}

string VelodyneNode::get_cloud_bin_name_by_index(int index)
{
	return cloud_bins_.get_filename(index);
}

void VelodyneNode::fill_frame_id(std_msgs::Header &header)
{
	header.frame_id = node_param_.frame_id();
}

void VelodyneNode::fill_stamp(std_msgs::Header &header, ros::Time stamp)
{
	if (node_param_.original_timestamp())
		header.stamp = timestamp_.to_ros_time();
	else	
		header.stamp = stamp;
}
	
REGISTER_NODE_CLASS(Velodyne);	
	
}//namespace kitti















