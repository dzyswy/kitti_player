#ifndef __KITTI_OXTS_DATA_H
#define __KITTI_OXTS_DATA_H



#include "kitti/data/sensor_data.h"



namespace kitti {
	
class OxtsData : public SensorData
{
public:
	OxtsData();
	bool empty();
	bool from_txt(const std::string &filename);
	void to_ros_msgs(sensor_msgs::Imu &value);

protected:
	bool parse_file(const std::string &filename);
	void fill_field(std::vector<std::string> &s);
	
public:
	bool empty_;

	double lat_;///@brief   latitude of the oxts-unit (deg)
	double lon_;///@brief   longitude of the oxts-unit (deg)
	double alt_;///@brief   altitude of the oxts-unit (m)
	double roll_;///@brief  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
	double pitch_;///@brief pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
	double yaw_;///@brief   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
	double vn_;///@brief    velocity towards north (m/s)
	double ve_;///@brief    velocity towards east (m/s)
	double vf_;///@brief    forward velocity, i.e. parallel to earth-surface (m/s)
	double vl_;///@brief    leftward velocity, i.e. parallel to earth-surface (m/s)
	double vu_;///@brief    upward velocity, i.e. perpendicular to earth-surface (m/s)
	double ax_;///@brief    acceleration in x, i.e. in direction of vehicle front (m/s^2)
	double ay_;///@brief    acceleration in y, i.e. in direction of vehicle left (m/s^2)
	double az_;///@brief    acceleration in z, i.e. in direction of vehicle top (m/s^2)
	double af_;///@brief    forward acceleration (m/s^2)
	double al_;///@brief    leftward acceleration (m/s^2)
	double au_;///@brief    upward acceleration (m/s^2)
	double wx_;///@brief    angular rate around x (rad/s)
	double wy_;///@brief    angular rate around y (rad/s)
	double wz_;///@brief    angular rate around z (rad/s)
	double wf_;///@brief    angular rate around forward axis (rad/s)
	double wl_;///@brief    angular rate around leftward axis (rad/s)
	double wu_;///@brief    angular rate around upward axis (rad/s)
	double pos_accuracy_;///@brief  velocity accuracy (north/east in m)
	double vel_accuracy_;///@brief  velocity accuracy (north/east in m/s)
	int navstat_;///@brief       navigation status (see navstat_to_string)
	int numsats_;///@brief       number of satellites tracked by primary GPS receiver
	int posmode_;///@brief       position mode of primary GPS receiver (see gps_mode_to_string)
	int velmode_;///@brief       velocity mode of primary GPS receiver (see gps_mode_to_string)
	int orimode_;///@brief       orientation mode of primary GPS receiver (see gps_mode_to_string)
	
	
};	
	
	
}//namespace kitti 














#endif

