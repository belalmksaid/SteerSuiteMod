
--[====[
solution "Adapter"

	configurations {"Release", "Debug"}
	configuration "Release"
		flags { "Optimize", "EnableSSE2","StaticRuntime", "NoMinimalRebuild", "FloatFast"}
	configuration "Debug"
		defines {"_DEBUG=1"}
		flags { "Symbols", "NoMinimalRebuild", "NoEditAndContinue"}


	platforms {"x64"}
	
	
	--]====]

project "steerSuiteAdapter"
	language "C++"
	kind "SharedLib"

	includedirs { 
		"../steerplugin/include",
		"../steeropt/include",
		"../steerlib/include",
		"../util/include",
		"../external",
		"../external/libcmaes",
		"../steersimlib/include",
		"../spacesyntax/include",
	}
	files { 
		"*.cpp",
		"../steeropt/include/*.h",
		"../steeropt/src/*.cpp",
		"../external/clipper/clipper.cpp",
	}
	links { 		
		"steerlib",
		"steersimlib",
		"util",
		"glfw",
		"libcmaes",
		"steerplugin",
		"steeropt",
		"spacesyntax",
	}

	targetdir ("../build/bin")
	-- buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",
			"-std=c++0x -ggdb"
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		libdirs { "lib" }
		links {
			"X11",
			"tinyxml",
			"dl",
			"pthread"
		}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		links { 
			"opengl32",
			"glu32",
		}
		files {
			"../external/tinyxml/*.h",
			"../external/tinyxml/*.cpp" 
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall -std=c++0x -ggdb" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"tinyxml",
			"dl",
			"pthread"
		}

project "steerSuiteAdapterCSharp"
	language "C#"
	kind "SharedLib"

	platforms {"x64"}
	includedirs { 
		
	}
	files { 
		"*.cs",
		"src/Properties.cs"
	}
	links { 		
		"steerSuiteAdapter",
	}

	targetdir ("../build/bin")
	-- buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux" }
		
	-- windows library cflags and libs
	configuration { "windows" }
		
	-- mac includes and libs
	configuration { "macosx" }
	
project "steerSuiteAdapterCSharpTester"
	language "C#"
	kind "ConsoleApp"

	platforms {"x64"}
	includedirs { 
		"*.cs",
	}
	files { 
		-- "*.cs",
		"src/*.cs"
	}
	links { 		
		"steerSuiteAdapterCSharp",
	}

	targetdir ("../build/bin")
	-- buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux" }
		
	-- windows library cflags and libs
	configuration { "windows" }
		
	-- mac includes and libs
	configuration { "macosx" }
		