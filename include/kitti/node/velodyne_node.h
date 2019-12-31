#ifndef __KITTI_VELODYNE_NODE_H
#define __KITTI_VELODYNE_NODE_H


#include "kitti/node/node.h"
#include "kitti/data/velodyne_points.h"

namespace kitti {
	
class VelodyneNode : public Node
{
public:
	VelodyneNode(ros::NodeHandle &node, const NodeParameter& param);	
	
	virtual int grab(int index);
	virtual int process();
	virtual int publish(ros::Time stamp);
	
protected:
	string get_cloud_bin_name_by_index(int index);
	void fill_frame_id(std_msgs::Header &header);
	void fill_stamp(std_msgs::Header &header, ros::Time stamp);
	
protected:
	FormattedFilename cloud_bins_;
	TimestampFile timestamps_;
	ros::Publisher cloud_pub_;
	
	 
	VelodynePoints points_;
	Timestamp timestamp_;
	
	
};	
	
	
	
	
}//namespace kitti
















#endif

