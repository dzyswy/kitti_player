#ifndef __KITTI_PLAYER_H
#define __KITTI_PLAYER_H


#include "kitti/node/node.h"

namespace kitti {
	
class Player
{
public:
	Player(const PlayerParameter& param);
	Player(const std::string& param_file);
	
protected:
	///@brief 从文件中加载player参数
	void load_player_parameter(const string& param_file, PlayerParameter* param);
	///@brief 用player的参数创建所有node和其他相关数据
	void init(const PlayerParameter& param);
	///@brief 检查player和node的重复字段参数，如果node没有被设置，则用player的参数覆盖设置。
	void cover_node_field();
	///@brief 创建所有node
	void create_nodes();
	///@brief 创建定时器，它每个一定时间触发，用来驱动去取一帧数据并发送ros消息。
	void create_timer();
	///@brief 定时器回调函数
	void timer_handle(const ros::TimerEvent& ev);
	///@brief 所有的节点读取一帧数据
	void grab(int index);
	///@brief 所有的节点处理一帧数据
	void process();
	///@brief 所有的节点发送一帧数据
	void publish(ros::Time stamp);
	
	
	long max_count()
	{
		return max_count_;
	}
	
	void set_max_count(long value)
	{
		max_count_ = value;
	}
	
protected:
	PlayerParameter player_param_;
	ros::NodeHandle ros_node_;
	int frame_count_;
	ros::Timer timer_;
	
	long max_count_;
	
	vector<shared_ptr<Node> > nodes_;
	vector<string> node_names_;
	map<string, int> node_names_index_;
};	
	
	
}//namespace kitti





















#endif


