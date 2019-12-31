#include "kitti/data/image_mat.h"






namespace kitti {


ImageMat::ImageMat()
{ 
}
 

bool ImageMat::empty()
{
    return image_.empty();
}

bool ImageMat::from_file(const std::string &filename)
{
    image_ = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
    return !image_.empty();
}




}//namespace kitti






















