#pragma once

#ifdef _WIN32
#define STEEROPTPLUG_API __declspec(dllexport)
#else
#define STEEROPTPLUG_API
#endif
