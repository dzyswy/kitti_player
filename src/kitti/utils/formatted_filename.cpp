#include "kitti/utils/formatted_filename.h"
#include <boost/algorithm/string.hpp>





namespace kitti {

using namespace std;

FormattedFilename::FormattedFilename(const std::string &path, const std::string &name_fmt, const std::string &suffix, int first_index)
{
    name_format_ = path + name_fmt;
    suffix_ = suffix;
    first_index_ = first_index;

}

FormattedFilename::FormattedFilename(const std::string &path, const std::string &subpath, const std::string &name_fmt, const std::string &suffix, int first_index = 0)
{
	name_format_ = path + subpath + name_fmt;
    suffix_ = suffix;
    first_index_ = first_index;
}   


std::string FormattedFilename::get_filename(int index)
{
    string filename = boost::str(boost::format(name_format_.c_str()) % index) + suffix_;
    DLOG(INFO) << filename;
	return filename;
}


}//namespace kitti





















