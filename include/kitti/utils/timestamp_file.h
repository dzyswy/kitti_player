#ifndef __TMSTMP_FILE_H
#define __TMSTMP_FILE_H



#include "kitti/common.h"
#include "kitti/data/timestamp.h"




namespace kitti {
	


class TimestampFile
{
public:	
	/**
	 * @brief      kitti时戳文件类
	 * @details    加载kitti时戳文件timestamps.txt，解析所有的时戳信息，并提供访问接口
	 *
	 * @param      file_name  时戳文件完整路径
	 */
	TimestampFile(const std::string &filename);
	TimestampFile(const std::string &path, const std::string &filename);
	///@brief 判断是否打开成功
	bool is_open();
	///@brief 返回时戳总数
	int count();
	/**
	* @brief	返回第index个时戳
	*
	* @param	t		[Timestamp]	时戳数据 
	* @param	index	[index] 	序号
	*
	* @return	成功返回0，错误返回-1
	*/
	int get_timestamp(Timestamp &value, int index);
	
private:
	bool parse_file(const std::string &filename);
	bool parse_line(Timestamp &value, const std::string &line);
	
private:
	int is_open_;
	std::vector<Timestamp> timestamps_;
};
	
	
	
}//namespace kitti




#endif



















