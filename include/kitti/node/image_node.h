#ifndef __KITTI_IMAGE_NODE_H
#define __KITTI_IMAGE_NODE_H





#include "kitti/node/node.h"
#include "kitti/data/image_mat.h"

namespace kitti {
	
class ImageNode : public Node
{
public:
	ImageNode(ros::NodeHandle &node, const NodeParameter& param);	
	
	virtual int grab(int index);
	virtual int process();
	virtual int publish(ros::Time stamp);
	
protected:
	string get_image_file_name_by_index(int index);
	void fill_frame_id(std_msgs::Header &header);
	void fill_stamp(std_msgs::Header &header, ros::Time stamp);

protected:
	FormattedFilename image_files_;
	TimestampFile timestamps_;
	image_transport::ImageTransport it_;
	image_transport::Publisher image_pub_;
	
	 
	ImageMat image_;
	Timestamp timestamp_;
	
};
















}//namespace kitti



#endif

