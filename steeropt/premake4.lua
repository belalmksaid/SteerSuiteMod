
project "steeropt"
	language "C++"
	kind "SharedLib"
	-- kind "StaticLib"
	
	includedirs { 
		"../steerlib/include",
		"../steersimlib/include",
		"../external",
		"../external/libcmaes",
		"../util/include",
		"../steerplugin/include",
		"./include",
		"../external/rockstar/cpp_code",
		"../spacesyntax/include",
	}
	files { 
		"include/*.h",
		-- "include/*.hpp",
		"src/*.cpp",
		-- "../external/libcmaes/src/esostrategy.cc"
		"../external/clipper/clipper.cpp",
	}
	excludes {
		"src/Main.cpp" 
	}
	links { 		
		"steerlib",
		-- "steersimlib",
		"glfw",
		"spacesyntax",
		"steerplugin",
		"util",
		"libcmaes",
		-- "tinyxml"
	}
	
	-- linux library cflags and libs
	configuration { "linux" }
		buildoptions { 
			-- "`pkg-config --cflags gl`",
			-- "`pkg-config --cflags glu`",
			"-std=c++0x -ggdb -fPIC -g",
			"-fopenmp",
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			-- "`pkg-config --libs gl`",
			-- "`pkg-config --libs glu`",
			-- " -lgsl -lgslcblas" 
		}
		libdirs { "lib" }
		links {
			-- "X11",
			-- "dl",
			"gomp",
			"pthread",
			"tinyxml"
		}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		links { 
			-- "opengl32",
			-- "glu32",
		}
		files {
			"../external/tinyxml/*.h",
			"../external/tinyxml/*.cpp" 
		}

	-- mac includes and libs
	configuration { "macosx" }
		-- kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		buildoptions { "-std=c++0x -ggdb -fPIC" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"-install_name @rpath/libsteeropt.dylib"
		}
		links { 
	--		"OpenGL.framework", 
		--	"Cocoa.framework",
		--	"dl",
			"pthread",
			"tinyxml"
		}
		
project "steeroptRun"
	language "C++"
	kind "ConsoleApp"
	-- kind "StaticLib"
	
	includedirs { 
		"../steerlib/include",
		"../steersimlib/include",
		"../external",
		"../external/libcmaes",
		"../util/include",
		"../steerplugin/include",
		"./include",
		"../external/rockstar/cpp_code",
		"../spacesyntax/include",
	}
	files { 
		-- "include/*.h",
		-- "include/*.hpp",
		"src/Main.cpp",
		-- "src/*.cpp",
		-- "../external/libcmaes/src/esostrategy.cc"
	}
	links { 		
		"steerlib",
		"steersimlib",
		"glfw",
		"steeropt",
	 	"spacesyntax",
		"libcmaes",
		"steerplugin",
		"util",
	}
	

	targetdir ( "../build/bin" )

	-- linux library cflags and libs
	configuration { "linux" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",
			"-std=c++0x -ggdb -g",
			"-fopenmp",
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			-- "`pkg-config --libs gl`",
			-- "`pkg-config --libs glu`",
			-- " -lgsl -lgslcblas" 
		}
		libdirs { "lib" }
		links {
			"dl",
			"gomp",
			"tinyxml",
			"pthread",
			"X11",
		}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		-- files {
		-- 	"../external/tinyxml/*.h",
		-- 	"../external/tinyxml/*.cpp" 
		-- }
		links { 
			"opengl32",
			"glu32",
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-std=c++0x -ggdb -g" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
		}
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			-- "tinyxml",
			"dl",
			"pthread"
		}
