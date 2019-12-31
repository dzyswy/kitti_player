#ifndef __KITTI_NODE_H
#define __KITTI_NODE_H

#include "kitti/common.h"
#include "kitti.pb.h"
#include "kitti/node/node_factory.h"
#include "kitti/data/sensor_data.h"
#include "kitti/data/timestamp.h"
#include "kitti/utils/formatted_filename.h"
#include "kitti/utils/timestamp_file.h"

namespace kitti {
	
class Node 
{
public:
	Node(ros::NodeHandle &node, const NodeParameter &param);
	virtual int grab(int index) = 0;
	virtual int process() = 0;
	virtual int publish(ros::Time stamp) = 0;
	virtual long max_count() {return max_count_;}
	
protected:
	virtual void set_max_count(long value) {max_count_ = value;}
	
protected:
	ros::NodeHandle &ros_node_;
	NodeParameter node_param_;
	long max_count_;
};	
	
	
	
}//namespace kitti
















#endif



