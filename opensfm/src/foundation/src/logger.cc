#include <glog/logging.h>
#include <foundation/types.h>

class GLogInitializationWrapper {
 public:
  GLogInitializationWrapper() { google::InitGoogleLogging("opensfm"); }
};

static GLogInitializationWrapper initializer;
