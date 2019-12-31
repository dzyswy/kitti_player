#include "kitti/utils/timestamp_file.h"
#include <boost/lexical_cast.hpp>


namespace kitti {
	
using namespace std;	
	
TimestampFile::TimestampFile(const std::string &filename)
{
	is_open_ = parse_file(filename);
}

TimestampFile::TimestampFile(const std::string &path, const std::string &filename)
{
	string fullname = path + filename;
	is_open_ = parse_file(fullname);
}

bool TimestampFile::is_open()
{
	return is_open_;
}

int TimestampFile::count()
{
	return timestamps_.size();
}

int TimestampFile::get_timestamp(Timestamp &t, int index)
{
	if ((index < 0) || (index >= count())) {
		LOG(ERROR) << "Failed to get timestamp: " << index << " max: " << count();
		return -1;	
	}
		 
	t = timestamps_[index];
	return 0;
}

bool TimestampFile::parse_file(const std::string &filename)
{
	ifstream is(filename);
	if (!is.is_open()) {
		return false;
	} 
	
	string line;
	while(getline(is, line))
	{
		Timestamp t;
		parse_line(t, line);
		timestamps_.push_back(t);
	}
	LOG(INFO) << "total timestamps: " << timestamps_.size();
	return true;
}

bool TimestampFile::parse_line(Timestamp &t, const std::string &line)
{
	if (line.size() < 28)
		return false;
	t.year_ = boost::lexical_cast<int>(line.substr(0, 4)) - 1900;
    t.mon_  = boost::lexical_cast<int>(line.substr(5, 2)) - 1;
    t.mday_ = boost::lexical_cast<int>(line.substr(8, 2));
    t.hour_ = boost::lexical_cast<int>(line.substr(11, 2));
    t.min_  = boost::lexical_cast<int>(line.substr(14, 2));
    t.sec_  = boost::lexical_cast<int>(line.substr(17, 2));
	t.nsec_ = boost::lexical_cast<int>(line.substr(20, 8));
	
	return true;
}	
	
}//namespace kitti






















