#pragma once

class GLogInitializationWrapper {
 public:
  GLogInitializationWrapper();
  static GLogInitializationWrapper& Instance();
};