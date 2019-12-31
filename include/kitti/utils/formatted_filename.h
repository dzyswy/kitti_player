#ifndef __FORMATTED_FILENAME_H
#define __FORMATTED_FILENAME_H


#include "kitti/common.h"





namespace kitti {




class FormattedFilename
{
public:
	/**
	 * @brief      格式化的文件名类
	 * @details    有些文件的名称是有规律的，一个序号相关的，本类提供获取任意序号的接口。
	 *
	 * @param      path         路径名称
	 * @param      name_fmt     文件名格式
	 * @param      suffix       后缀
	 * @param      first_index  起始序号
	 */
    FormattedFilename(const std::string &path, const std::string &name_fmt, const std::string &suffix, int first_index = 0);
    /**
	* @brief	返回第index个格式化文件名
	*
	* @param	index	[index] 	序号
	*
	* @return	返回格式化文件名
	*/
    std::string get_filename(int index);


private:
    std::string name_format_;
    std::string suffix_;
    int first_index_;
};




}//namespace kitti




#endif











