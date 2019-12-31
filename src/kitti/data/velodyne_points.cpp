#include "kitti/data/velodyne_points.h"






namespace kitti {


VelodynePoints::VelodynePoints()
{
	points_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    empty_ = true;
}

VelodynePoints::~VelodynePoints()
{
}

bool VelodynePoints::empty()
{
    return empty_;
}

bool VelodynePoints::from_bin(const std::string &filename)
{
    if (parse_file(filename))
        empty_ = false;
    else
        empty_ = true;
    return empty_;
}

void VelodynePoints::to_ros_msgs(sensor_msgs::PointCloud2 &value)
{
	pcl::toROSMsg(*points_, value);
}

bool VelodynePoints::parse_file(const std::string &filename)
{
    fstream input(filename.c_str(), ios::in | ios::binary);
    if (!input.good()) {
        return  false;
    }
        
    input.seekg(0, ios::beg);
    points_->clear();
    for (int i = 0; input.good() && !input.eof(); i++)
    {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points_->push_back(point);
    }
    input.close();
    return true;
}



}//namespace kitti






















