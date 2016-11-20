

project "steerdemo"
	language "C++"
	kind "WindowedApp"
	includedirs { 
		"../steerlib/include",
		"../steerdemo/include",
		"../steersimlib/include",
		"../external",
		"../util/include",
		"../pprAI/include",
		"../rvo2AI/include",
		-- "../external/qhull/src"
		
	}
	files { 
		"../steerdemo/include/*.h",
		"../steerdemo/src/*.cpp"
	}
	links { 		
		"steerlib",
		"steersimlib",
		"util",
		"glfw",
		--"AntTweakBar"
	}

	targetdir "bin"
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
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
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

