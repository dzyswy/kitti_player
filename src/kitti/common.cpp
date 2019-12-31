#include "kitti/common.h"




namespace kitti {
	
void GlobalInit(int* pargc, char*** pargv) 
{
  // Google logging.
  ::google::InitGoogleLogging(*(pargv)[0]);
  // Provide a backtrace on segfault.
  ::google::InstallFailureSignalHandler();
}
	
	
	
}//namespace kitti
















