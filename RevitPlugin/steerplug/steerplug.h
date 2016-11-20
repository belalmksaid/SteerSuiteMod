// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the STEERPLUG_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// STEERPLUG_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef STEERPLUG_EXPORTS
#define STEERPLUG_API __declspec(dllexport)
#else
#define STEERPLUG_API __declspec(dllimport)
#endif

extern "C" {
	STEERPLUG_API double optimize(double a);
	STEERPLUG_API double optimize2(double const* a, size_t size, size_t const * faces, size_t face_count);
}
// This class is exported from the steerplug.dll
class STEERPLUG_API Csteerplug {
public:
	Csteerplug(void);
	// TODO: add your methods here.
};

extern STEERPLUG_API int nsteerplug;

STEERPLUG_API int fnsteerplug(void);
