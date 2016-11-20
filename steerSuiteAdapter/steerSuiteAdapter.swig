
 /* steersuiteadapter.swig */
 
 %module steerSuiteAdapter
 %{
 /* Put header files here or function declarations like below */
 #define SWIG_FILE_WITH_INIT
 #include <vector>
 
 #include "../steerlib/include/Globals.h"
 #include "../steerlib/include/util/Geometry.h"
 #include "../steerlib/include/interfaces/ModuleInterface.h"
 #include "../steerplugin/include/SteerPlugin.h"
 #include "../steerplugin/include/SimWorld.h"
 #include "../steerplugin/include/SteerSuite.h"
 #include "../steeropt/include/SteerOptPlugin.h"
 #include "../steeropt/include/SteerSimOptimize.h"
 #include "../steeropt/include/CMAOptimize.h"
 #include "../steeropt/include/OptimizationParameters.h"
 #include "../steeropt/include/OptimizationConfiguration.h"
 #include "../steeropt/include/Graph.h"
 #include "../external/clipper/clipper.hpp"
 #include "../steeropt/include/RoundRobinOptimization.h"
 #include "../steeropt/include/HierarchicalOptimization.h"
 %}
 
 %include "std_string.i"
 %include "std_vector.i"
 %include "std_map.i"
 // %include "std_set.i"
 
 // Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(Vector_SizeT) vector<size_t>;
   %template(FloatVector) vector<float>;
   %template(PointVector) vector<Util::Point>;
   // %template(SizeTSet) set<size_t>;
   %template(PointVectorVector) vector<vector<Util::Point>>;
   %template(OptionDictionaryMap) map<string, string>;
}

// Rename some operators
/*
%rename(addPoint) operator+(const Point);
%rename(addScalar) operator+(float ,const Point);
%rename(addVector) operator+(const Vector&);
%rename(addScalar) operator+(float ,const Vector);
*/

%rename(at) operator[];
%rename(add) operator+;
%rename(multiply) operator*;
%rename(divide) operator/;
%rename(subtract) operator-;
%rename(lessThan) operator<;
%rename(equal) operator==;
%rename(notEqual) operator!=;
%rename(incrementAdd) operator+=;
%rename(incrementSubtract) operator-=;
%rename(incrementMultiply) operator*=;
%rename(incrementDivide) operator/=;

// %ignore  std::ostream & operator<<(std::ostream , const Vector &);
%ignore operator<<;
// %rename(Util_Point) Util::Point;
// %rename(Util_Vector) Util:: Vector;

%include "../steerlib/include/Globals.h"
%include "../steerlib/include/util/Geometry.h"
%include "../steerlib/include/interfaces/ModuleInterface.h"
%include "../steerplugin/include/SteerPlugin.h"
%include "../steerplugin/include/SimWorld.h"
%include "../steerplugin/include/SteerSuite.h"
%include "../steeropt/include/SteerOptPlugin.h"
%include "../steeropt/include/SteerSimOptimize.h"
%include "../steeropt/include/CMAOptimize.h"
%include "../steeropt/include/OptimizationParameters.h"
%include "../steeropt/include/OptimizationConfiguration.h"
%include "../steeropt/include/Graph.h"
%include "../external/clipper/clipper.hpp"
%include "../steeropt/include/RoundRobinOptimization.h"
%include "../steeropt/include/HierarchicalOptimization.h"
 
 
 