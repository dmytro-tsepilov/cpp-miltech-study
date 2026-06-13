#pragma once
#include <iostream>

#define ENABLE_LOG    1
#define ENABLE_DEBUG  1

// Set to 1 to enable HTTP-based config/target loading
// Set to 0 or comment out to remove all HTTP code from compilation
#ifndef ENABLE_HTTP
#define ENABLE_HTTP 0
#endif

#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif
