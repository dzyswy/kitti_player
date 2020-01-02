#include "kitti/data/oxts_data.h"




namespace kitti {
	
using namespace std;	
	
OxtsData::OxtsData()
{
    empty_ =false;
}

bool OxtsData::empty()
{
    return empty_;
}

bool OxtsData::from_txt(const std::string &filename)
{
    if (parse_file(filename)) 
        empty_ = false;
    else {
		LOG(WARNING) << "oxts data is empty";
		empty_ = true;
	}
    return empty_;
}

void OxtsData::to_ros_msgs(sensor_msgs::Imu &value)
{
	value.linear_acceleration.x = ax_;
	value.linear_acceleration.y = ay_;
	value.linear_acceleration.z = az_;
	
	value.angular_velocity.x = vf_;
	value.angular_velocity.y = vl_;
	value.angular_velocity.z = vu_;
	
	tf::Quaternion q = tf::createQuaternionFromRPY(roll_, pitch_, yaw_);
    value.orientation.x = q.getX();
    value.orientation.y = q.getY();
    value.orientation.z = q.getZ();
    value.orientation.w = q.getW();
}


bool OxtsData::parse_file(const std::string &filename)
{
    ifstream is(filename);
    if (!is.is_open()) {
        printf("Failed to open oxts file: %s\n", filename.c_str());
        return false;
    }
 
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep {" "};

    string line = "";

    getline(is, line);
    tokenizer tok(line, sep);
    vector<string> s(tok.begin(), tok.end());
    if (s.size() < 30)
    {
        printf("oxts file item less than 30, total %d\n", s.size());
        return false;
    }

    fill_field(s);

    return true;
}


void OxtsData::fill_field(std::vector<std::string> &s)
{
    lat_ = boost::lexical_cast<double>(s[0]);
    lon_ = boost::lexical_cast<double>(s[1]);
    alt_ = boost::lexical_cast<double>(s[2]);
    roll_ = boost::lexical_cast<double>(s[3]);
    pitch_ = boost::lexical_cast<double>(s[4]);
    yaw_ = boost::lexical_cast<double>(s[5]);
    vn_ = boost::lexical_cast<double>(s[6]);
    ve_ = boost::lexical_cast<double>(s[7]);
    vf_ = boost::lexical_cast<double>(s[8]);
    vl_ = boost::lexical_cast<double>(s[9]);
    vu_ = boost::lexical_cast<double>(s[10]);
    ax_ = boost::lexical_cast<double>(s[11]);
    ay_ = boost::lexical_cast<double>(s[12]);
    az_ = boost::lexical_cast<double>(s[13]);
    af_ = boost::lexical_cast<double>(s[14]);
    al_ = boost::lexical_cast<double>(s[15]);
    au_ = boost::lexical_cast<double>(s[16]);
    wx_ = boost::lexical_cast<double>(s[17]);
    wy_ = boost::lexical_cast<double>(s[18]);
    wz_ = boost::lexical_cast<double>(s[19]);
    wf_ = boost::lexical_cast<double>(s[20]);
    wl_ = boost::lexical_cast<double>(s[21]);
    wu_ = boost::lexical_cast<double>(s[22]);
    pos_accuracy_ = boost::lexical_cast<double>(s[23]);
    vel_accuracy_ = boost::lexical_cast<double>(s[24]);
    navstat_ = std::round(boost::lexical_cast<double>(s[25]));
    numsats_ = std::round(boost::lexical_cast<double>(s[26]));
    posmode_ = std::round(boost::lexical_cast<double>(s[27]));
    velmode_ = std::round(boost::lexical_cast<double>(s[28]));
    orimode_ = std::round(boost::lexical_cast<double>(s[29]));

}	
	
}//namespace kitti

















