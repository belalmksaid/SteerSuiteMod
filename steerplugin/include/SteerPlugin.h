#pragma once

#ifdef _WIN32
#define STEERPLUG_API __declspec(dllexport)
#else
#define STEERPLUG_API 
#endif