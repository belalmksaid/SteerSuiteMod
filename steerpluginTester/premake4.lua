
project "steerpluginTester"
	language "C++"
	kind "ConsoleApp"

	includedirs { 
		"../steerlib/include",
		"../steerplugin/include",
		"../steersimlib/include",
		"../external",
		"../external/libcmaes",
		"../external/boost",
		"../util/include",
		"../spacesyntax/include",
		"../steeropt/include",
	}
	files { 
		"include/*.h",
		"src/*.cpp"
	}
	links { 		
		"steerplugin",
		"steerlib",
		"steersimlib",
		"util",
		"spacesyntax",
		"steeropt",
		"libcmaes",
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
			"gomp",
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
