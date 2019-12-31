#include "kitti/player.h"




using namespace std;






int main(int argc, char *argv[])
{
	FLAGS_alsologtostderr = true;
	
	if (argc != 2)
	{
		printf("usage: %s *.prototxt\n", argv[0]);
		return -1;
	}	
	string proto_name = argv[1];
	
    ros::init(argc, argv, "kitti_player");   
	kitti::GlobalInit(&argc, &argv);
	
    kitti::Player player(proto_name);

    ROS_INFO("\033[1;32m---->\033[0m Kitti Player Started.");

    ros::spin();
    return 0;
    
}





















