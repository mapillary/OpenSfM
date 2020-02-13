#include "../logger.h"
#include <glog/logging.h>


GLogInitializationWrapper::GLogInitializationWrapper(){
  google::InitGoogleLogging("opensfm");
}

GLogInitializationWrapper& GLogInitializationWrapper::Instance()
{
  static GLogInitializationWrapper initializer;
  return initializer;
}

