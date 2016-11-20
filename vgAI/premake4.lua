
project "vgAI"
	language "C++"
	kind "SharedLib"

	includedirs { 
		"../steerlib/include",
		"../vgAI/include",
		"../external",
		"../external/Eigen3.2.7",
		"../util/include",
		"../spacesyntax/include",
	}
	files { 
		"../vgAI/include/*.h",
		"../vgAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util",
		"spacesyntax"
	}

	-- targetdir "bin"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
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

	-- mac includes and libs
	configuration { "macosx" }
		-- kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
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
