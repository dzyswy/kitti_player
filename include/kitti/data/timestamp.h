#ifndef __KITTI_TIMESTAMP_H_
#define __KITTI_TIMESTAMP_H_



#include "kitti/common.h"


namespace kitti {


class Timestamp
{
public:
    Timestamp();
	ros::Time to_ros_time();

public:
    int year_;
    int mon_;
    int mday_;
    int hour_;
    int min_;
    int sec_;
    int nsec_;
};


}//namespace kitti


















#endif

