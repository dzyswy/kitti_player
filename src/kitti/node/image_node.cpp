#include "kitti/node/image_node.h"




namespace kitti {
	
ImageNode::ImageNode(ros::NodeHandle &node, const NodeParameter& param)
	: Node(node, param),
	image_files_(param.dataset_root_path(), param.subset_path(), "/data/%010d", param.suffix()),
	timestamps_(param.dataset_root_path(), param.subset_path(), "/timestamps.txt"),
	it_(node)
{
	CHECK(timestamps_.is_open()) << "Failed to open timestamp file";
	
	set_max_count(timestamps_.count());
	image_pub_ = it_.advertise(param.topic(), 1);
}

int ImageNode::grab(int index)
{
	image_.from_file(get_image_file_name_by_index(index));
	if (image_.empty())
		return -1;
	
	if (!timestamps_.get_timestamp(timestamp_, index))
		return -1;
	
	return 0;
}

int ImageNode::process()
{
	
	return 0;
}

int ImageNode::publish(ros::Time stamp)
{
	if (image_.empty())
		return -1;
	
	sensor_msgs::ImagePtr msgs = cv_bridge::CvImage(std_msgs::Header(), node_param_.image_spec_param().encoding(), image_.image_).toImageMsg();
	fill_frame_id(msgs->header);
	fill_stamp(msgs->header, stamp);
	image_pub_.publish(msgs);	
	return 0;
}

string ImageNode::get_image_file_name_by_index(int index)
{
	return image_files_.get_filename(index);
}

void ImageNode::fill_frame_id(std_msgs::Header &header)
{
	header.frame_id = node_param_.frame_id();
}

void ImageNode::fill_stamp(std_msgs::Header &header, ros::Time stamp)
{
	if (node_param_.original_timestamp())
		header.stamp = timestamp_.to_ros_time();
	else	
		header.stamp = stamp;
}
	
REGISTER_NODE_CLASS(Image);	
	
	
}//namespace kitti




























