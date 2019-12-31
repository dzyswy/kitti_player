#ifndef __KITTI_NODE_FACTORY_H
#define __KITTI_NODE_FACTORY_H


#include "kitti/common.h"
#include "kitti/node/node.h"


namespace kitti {
	
class Node;

class NodeRegistry
{
public:
	typedef shared_ptr<Node> (*Creator) (ros::NodeHandle&, const NodeParameter&);
	typedef std::map<string, Creator> CreatorRegistry;
	
	///@brief 分配并返回唯一的创建函数注册表
	static CreatorRegistry & Registry() 
	{
		static CreatorRegistry *g_registry_ = new CreatorRegistry();
		return *g_registry_;
	}
	
	///@brief 向注册表内注册创建函数
	static void AddCreator(const string& type, Creator creator)
	{
		CreatorRegistry& registry = Registry();
		CHECK_EQ(registry.count(type), 0)
			<< "Node type " << type << " already registered.";
		registry[type] = creator;
	}
	
	///@brief 从param参数创建Node类
	static shared_ptr<Node> CreateNode(ros::NodeHandle &node, const NodeParameter& param)
	{
		const string& type = param.type();
		CreatorRegistry& registry = Registry();
		CHECK_EQ(registry.count(type), 1) << "Unknown node type: " << type
			<< " (known types: " << NodeTypeListString() << ")";
		return registry[type](node, param);
	}
	
	///@brief 查询注册表内的所有注册项名称
	static vector<string> NodeTypeList()
	{
		CreatorRegistry& registry = Registry();
		vector<string> node_types;
		for (typename CreatorRegistry::iterator iter = registry.begin(); iter != registry.end(); iter++)
		{
			node_types.push_back(iter->first);
		}
		
		return node_types;
	}
	
private:
	NodeRegistry()
	{
		
	}
	
	static string NodeTypeListString()
	{
		vector<string> node_types = NodeTypeList();
		string node_types_str;
		for (auto it = node_types.begin(); it != node_types.end(); it++)
		{
			if (it != node_types.begin())
				node_types_str += ", ";
			node_types_str += *it;
		}	
		return node_types_str;
	}
};


///@brief 用于注册Node的类
class NodeRegisterer
{
public:
	NodeRegisterer(const string& type, shared_ptr<Node> (*creator) (ros::NodeHandle&, const NodeParameter&))
	{
		NodeRegistry::AddCreator(type, creator);
	}
};

///@brief 定义一个静态注册类
#define REGISTER_NODE_CREATOR(type, creator)                                  	\
	static NodeRegisterer g_creator_f_##type(#type, creator)     				\

///@brief 根据Node子类的前缀名称，创建一个全局函数，用于创建这个子类，并将其注册到注册表中
#define REGISTER_NODE_CLASS(type)                                             	\
	shared_ptr<Node> Creator_##type##Node(ros::NodeHandle& node, const NodeParameter& param) 	\
	{                                                                            	\
		return shared_ptr<Node>(new type##Node(node, param));           	\
	}                                                                            	\
	REGISTER_NODE_CREATOR(type, Creator_##type##Node)


	
}//namespace kitti



















#endif

