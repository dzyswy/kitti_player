#include "kitti/data/timestamp.h"






namespace kitti {



Timestamp::Timestamp()
{
    year_ = 0;
    mon_ = 0;
    mday_ = 0;
    hour_ = 0;
    min_ = 0;
    sec_ = 0;
    nsec_ = 0;
}

ros::Time Timestamp::to_ros_time()
{
	ros::Time stamp;
	
	struct tm t = {0};  // Initalize to all 0's
    t.tm_year = year_;
    t.tm_mon  = mon_;
    t.tm_mday = mday_;
    t.tm_hour = hour_;
    t.tm_min  = min_;
    t.tm_sec  = sec_;
    t.tm_isdst = -1;
    time_t timeSinceEpoch = mktime(&t);
    stamp.sec  = timeSinceEpoch;
    stamp.nsec = nsec_;
	
	return stamp;
}


}//namespace kitti




















