#ifndef __VELODYNE_POINTS_H
#define __VELODYNE_POINTS_H



#include "kitti/data/sensor_data.h"

namespace kitti {



class VelodynePoints : public SensorData
{
public:
    VelodynePoints();
    ~VelodynePoints();
    bool from_bin(const std::string &filename);
    bool empty();
	void to_ros_msgs(sensor_msgs::PointCloud2 &value);

private:
    bool parse_file(const std::string &filename);    

public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr points_;
    bool empty_;
};




}//namespace kitti





















#endif

