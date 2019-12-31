#ifndef __KITTI_IMAGE_MAT_H
#define __KITTI_IMAGE_MAT_H



#include "kitti/data/sensor_data.h"

namespace kitti {



class ImageMat : public SensorData
{
public:
    ImageMat(); 
    bool from_file(const std::string &filename);
    bool empty();
  

public:
    cv::Mat image_;
};




}//namespace kitti





















#endif

