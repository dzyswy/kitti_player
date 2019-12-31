#include "kitti/player.h"
#include "kitti/utils/io.h"






namespace kitti {

Player::Player(const PlayerParameter& param)
{
	init(param);
}

Player::Player(const std::string& param_file)
{
	PlayerParameter param;
	load_player_parameter(param_file, &param);
	init(param);
}	

void Player::load_player_parameter(const string& param_file, PlayerParameter* param)
{
	CHECK(ReadProtoFromTextFile(param_file, param))
      << "Failed to parse PlayerParameter file: " << param_file;
}

	
void Player::init(const PlayerParameter& param)
{
	max_count_ = -1;
	frame_count_ = 0;
	
	player_param_.CopyFrom(param);
	cover_node_field();
	create_nodes();
	create_timer();
}	
	
	
void Player::cover_node_field()
{
	for (int i = 0; i < player_param_.node_size(); i++)
	{
		///如果player设置了dataset_root_path字段，且node未设置该字段，则赋值node的该字段
		const NodeParameter& node_param = player_param_.node(i);
		if (player_param_.has_dataset_root_path() && !node_param.has_dataset_root_path()) 
			node_param.set_dataset_root_path(player_param_.dataset_root_path());
		
		///如果player设置了frame_id字段，且node未设置该字段，则赋值node的该字段
		if (player_param_.has_frame_id() && !node_param.has_frame_id()) 
			node_param.set_frame_id(player_param_.frame_id());
		
		///如果player设置了original_timestamp字段，且node未设置该字段，则赋值node的该字段
		if (player_param_.has_original_timestamp() && !node_param.has_original_timestamp()) 
			node_param.set_original_timestamp(player_param_.original_timestamp());	
	}	
}	
	
void Player::create_nodes()
{
	for (int i = 0; i <player_param_.node_size(); i++)
	{
		const NodeParameter& node_param = player_param_.node(i);
		nodes_.push_back(NodeRegistry::CreateNode(ros_node_, node_param));
		node_names_.push_back(node_param.name());
	}
	
	for (int i = 0; i < node_names_.size(); i++)
	{
		node_names_index_[node_names_[i]] = i;
		if (nodes_[i]->max_count() > max_count())
			set_max_count(nodes_[i]->max_count());
	}	
}

void Player::create_timer()
{
	timer_ = ros_node_.createTimer(ros::Duration(player_param_.frequency()), &Player::timer_handle, this);
}

void Player::timer_handle(const ros::TimerEvent& ev)
{
	ros::Time stamp = ros::Time::now();
	
	int index = frame_count_ % max_count();
	grab(index);
	process();
	publish(stamp);

	ROS_INFO_STREAM("frame: " << index);
	
	frame_count_++;
	if (frame_count_ >= max_count())
	{
		if (player_param_.loop())
		{
			if ((frame_count_ % max_count()) == 0)
				ROS_INFO_STREAM("touch max count, loop mode, index restart!");
		}	
			
		else
		{
			ROS_INFO_STREAM("touch max count, not loop mode, player will shutdown!");
			ros::shutdown();
		}	
	}	
		
}	

void Player::grab(int index)
{
	for (int i = 0; i < nodes_.size(); i++)
	{
		nodes_[i]->grab(index);
	}	
}

void Player::process()
{
	for (int i = 0; i < nodes_.size(); i++)
	{
		nodes_[i]->process();
	}	
}

void Player::publish(ros::Time stamp)
{
	for (int i = 0; i < nodes_.size(); i++)
	{
		nodes_[i]->publish(stamp);
	}	
}
	
	
}//namespace kitti













