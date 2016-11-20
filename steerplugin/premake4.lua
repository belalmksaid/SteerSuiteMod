
project "steerplugin"
	language "C++"
	kind "SharedLib"

	includedirs { 
		"../steerlib/include",
		"../steerplugin/include",
		"../steersimlib/include",
		"../external",
		"../util/include",
		"../spacesyntax/include"
	}
	files { 
		"include/*.h",
		"../external/clipper/clipper.cpp",
		-- "../external/clipper/clipper_extra.hpp",
		"src/*.cpp"
	}
	links { 		
		"steerlib",
		"steersimlib",
		"util",
		"glfw",
		"spacesyntax"
	}

	-- targetdir "bin"
	buildoptions("-std=c++0x -ggdb -g" )	

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
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-std=c++0x -ggdb -fPIC -g" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"-install_name @rpath/libsteerplugin.dylib"	
		}
		links { 
			-- "OpenGL.framework", 
			-- "Cocoa.framework",
			"tinyxml",
			"dl",
			"pthread"
		}
