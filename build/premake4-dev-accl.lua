--
-- premake4 file to build SteerSuite
-- http://steersuite.cse.yorku.ca
--

local action = _ACTION or ""
local todir = "./" .. action



solution "steersuite"
	configurations { 
		"Debug",
		"Release"
	}
	location (todir)
	linkoptions { 
		-- "-Wl,-rpath,./lib",
		"-Wl,-rpath," .. path.getabsolute("lib")
	}
	-- libdirs { "../lib" }

	-- extra warnings, no exceptions or rtti
	flags { 
		"ExtraWarnings",
--		"FloatFast",
--		"NoExceptions",
--		"NoRTTI",
		"Symbols"
	}
	defines { "ENABLE_GUI", "ENABLE_GLFW" }

	-- debug configs
	configuration "Debug*"
		defines { "DEBUG" }
		targetdir ( todir .. "lib" )
		flags {
			"Symbols",
			Optimize = Off
		}
 
 	-- release configs
	configuration "Release*"
		defines { "NDEBUG" }
		flags { "Optimize" }
		targetdir ( todir .. "lib" )

	-- windows specific
	configuration "windows"
		defines { "WIN32", "_WINDOWS" }

	configuration { "macosx" }
        buildoptions { "-stdlib=libc++" }
		linkoptions { "-stdlib=libc++" }
		links {
	        "OpenGL.framework",
        }
		
	if os.get() == "macosx" then
		premake.gcc.cc = "clang"
		premake.gcc.cxx = "clang++"
		-- buildoptions("-std=c++0x -ggdb -stdlib=libc++" )
	end

project "steersim"
	language "C++"
	kind "WindowedApp"
	includedirs { 
		"../steerlib/include",
		"../steersim/include",
		"../steersimlib/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../steersim/include/*.h",
		"../steersim/src/*.cpp"
	}
	links { 		
		"steerlib",
		"steersimlib",
		"tinyxml",
		"util",
		"AntTweakBar",
		"glfw",
		"dl",
		"pthread"
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
		}

	-- windows library cflags and libs
	configuration { "windows" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/x86" }
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
		}

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
		"tinyxml",
		"util",
		"AntTweakBar",
		"glfw",
		"dl",
		"pthread"
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
		}

	-- windows library cflags and libs
	configuration { "windows" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/x86" }
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
		}

project "steersimlib"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../steersimlib/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../steersimlib/include/*.h",
		"../steersimlib/src/*.cpp"
	}
	links { 		
		"steerlib",
		"tinyxml",
		"util",
		"AntTweakBar",
		"glfw",
		"dl",
		"pthread",
	--	"Xrandr"
	}

	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",
			"-fPIC"
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`",
			"-fPIC"
		}
		libdirs { "lib" }
		links {
			"Xrandr",
			"X11",
		}

	-- windows library cflags and libs
	configuration { "windows" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/x86" }
		links { 
			"opengl32",
			"glu32",
		}

	configuration { "macosx" }
		linkoptions { 
			"-install_name @rpath/libsteersimlib.dylib"
		}

--[====[
	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
		}
--]====]
		
project "tinyxml"
        language "C++"
        kind "SharedLib"
        includedirs {
                "../external/tinyxml"
        }
        files {
                "../external/tinyxml/*.h",
                "../external/tinyxml/*.cpp"
        }
        targetdir "lib"
		buildoptions("-std=c++0x -ggdb" )	
        configuration { "macosx" }
			linkoptions { 
				"-install_name @rpath/libtinyxml.dylib"
		}
			
project "util"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../util/include"
	}
	files { 
		"../util/include/*.h",
		"../util/src/*.cpp"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
		linkoptions { 
			"-install_name @rpath/libutil.dylib"
	}

project "steerlib"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include" ,
		"../external" ,
		"../steerlib/include/util", 
		"../util/include" 
		
	}
	files { 
		"../steerlib/include/*.h",
		
--		"../steerlib/include/util/*.h",
		"../steerlib/src/*.cpp" 
	}
	
	links { 
		"tinyxml",
		"util"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
		links {
			"OpenGL.framework",
		}
		linkoptions { 
			"-install_name @rpath/libsteerlib.dylib"
		}

project "glfw"
	kind "SharedLib"
    	language "C"
   	includedirs { 
		"../external/glfw/include",
		"../external/glfw/include/GL",
		"../external/glfw/lib",
		-- "../external/glfw/lib/x11"
	}
	files { 
	--	"../external/glfw/lib/x11/*.h",
	--	"../external/glfw/lib/x11/*.c",
		"../external/glfw/lib/*.h",
		"../external/glfw/lib/*.c"
	}

	links { 
		--"X11",
		"pthread",
		-- "Xrandr",
		-- "Xxf86vm",
		-- "Xext",
		-- "m",
		-- "GLU",
		-- "GL"
	}

	targetdir "lib"

    configuration {"linux"}
        files { 
		"../external/glfw/lib/x11/*.c",
		 "../external/glfw/x11/*.h" 
	}
        includedirs { "../external/glfw/lib/x11" }
        defines { "_GLFW_USE_LINUX_JOYSTICKS", "_GLFW_HAS_XRANDR", "_GLFW_HAS_PTHREAD" ,"_GLFW_HAS_SCHED_YIELD", "_GLFW_HAS_GLXGETPROCADDRESS" }
        buildoptions { 
			"-pthread",
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",  
			"-fPIC",
		}
       
    configuration {"windows"}
        files { 
		"../external/glfw/lib/win32/*.c",
		"../external/glfw/win32/*.h" 
	}
        includedirs { "../external/glfw/lib/win32" }
        defines { "_GLFW_USE_LINUX_JOYSTICKS", "_GLFW_HAS_XRANDR", "_GLFW_HAS_PTHREAD" ,"_GLFW_HAS_SCHED_YIELD", "_GLFW_HAS_GLXGETPROCADDRESS" }
       
    configuration {"macosx"}
        files { "../external/glfw/lib/cocoa/*.c",
		 "../external/glfw/lib/cocoa/*.h",
		 "../external/glfw/lib/cocoa/*.m" 
	}
        includedirs { "../external/glfw/lib/cocoa" }
        defines { }
--	removebuildoptions "-std=c++0x"
        linkoptions { 
		"-framework OpenGL", 
		"-framework Cocoa", 
		"-framework IOKit", 
                "-install_name @rpath/libglfw.dylib"
	}
	buildoptions {
                        "-fPIC",
                }


project "AntTweakBar"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../external/AntTweakBar/include",
		"../external/AntTweakBar/src",
	}
	files { 
		-- "../external/AntTweakBar/src/*.h",
		"../external/AntTweakBar/src/TwColors.cpp",
		"../external/AntTweakBar/src/TwFonts.cpp",
		"../external/AntTweakBar/src/TwOpenGL.cpp",
		"../external/AntTweakBar/src/TwOpenGLCore.cpp",
		"../external/AntTweakBar/src/TwBar.cpp",
		"../external/AntTweakBar/src/TwMgr.cpp",
		"../external/AntTweakBar/src/TwPrecomp.cpp", 
		"../external/AntTweakBar/src/LoadOGL.cpp", 
		"../external/AntTweakBar/src/LoadOGLCore.cpp", 
		"../external/AntTweakBar/src/TwEventGLFW.c", 
		"../external/AntTweakBar/src/TwEventGLUT.c", 
		"../external/AntTweakBar/src/TwEventSDL.c", 
		"../external/AntTweakBar/src/TwEventSDL12.c", 
		"../external/AntTweakBar/src/TwEventSDL13.c", 
		"../external/AntTweakBar/src/TwEventSFML.cpp", 
	--	"../external/AntTweakBar/src/TwEventX11.c"
	}
	targetdir "lib"	
	buildoptions("-std=c++0x -ggdb" )	

	configuration { "debug" }
		defines {"_DEBUG" }	
	
	configuration { "linux" }
		files { 
			"../external/AntTweakBar/src/TwEventX11.c"
		}
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		defines {"__PLACEMENT_NEW_INLINE", "_UNIX" }

	-- windows library cflags and libs
	configuration { "windows" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/x86" }
		links { 
			"opengl32",
			"glu32",
		}

		defines { "ANT_WINDOWS"}

	-- mac includes and libs
	configuration { "macosx" }
		-- kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		--files { 
		--	"../external/AntTweakBar/src/TwEvent.c"
		--}
		includedirs { 
			"/Developer/SDKs/MacOSX10.5.sdk/System/Library/Frameworks/OpenGL.framework/Headers/",
        	"/Developer/SDKs/MacOSX10.5.sdk/System/Library/Frameworks/GLUT.framework/Headers/",
	        "/Developer/SDKs/MacOSX10.5.sdk/System/Library/Frameworks/AppKit.framework/Headers/",
			"/usr/local/include",
			"/usr/X11R6/include",
			"/usr/include"

		}
		buildoptions { "-Wall -ObjC++" }
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
			"AppKit.framework",
		}
		defines { "_MACOSX", "__PLACEMENT_NEW_INLINE"}

project "simpleAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../simpleAI/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../simpleAI/include/*.h",
		"../simpleAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util"
	}
	targetdir "lib"	
	buildoptions("-std=c++0x -ggdb" )	
	
project "ccAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../ccAI/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../ccAI/include/*.h",
		"../ccAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	
project "rvo3dAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../rvo3dAI/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../rvo3dAI/include/*.h",
		"../rvo3dAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	
project "hidacAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../hidacAI/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../hidacAI/include/*.h",
		"../hidacAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	
project "egocentricAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../egocentricAI/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../egocentricAI/include/*.h",
		"../egocentricAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	
project "footstepAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../footstepAI/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../footstepAI/include/*.h",
		"../footstepAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
		links {
			"OpenGL.framework",
                }
	

	
project "sfAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../socialForcesAI/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../socialForcesAI/include/*.h",
		"../socialForcesAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util"
	}
	targetdir "lib"	
	buildoptions("-std=c++0x -ggdb" )	
	
project "rvo2dAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../rvo2AI/include",
		"../external",
		"../util/include",
		"../kdtree/include", 
		"../meshdatabase/include",
		"../acclmesh/include"
	}
	files { 
		"../rvo2AI/include/*.h",
		"../rvo2AI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util",
		"meshdatabase"
	}
	targetdir "lib"	
	buildoptions("-std=c++0x -ggdb" )	

project "pprAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../pprAI/include",
		"../external",
		"../util/include",
	}
	files { 
		"../pprAI/include/*.h",
		"../pprAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util"
	}
	targetdir "lib"	
	buildoptions("-std=c++0x -ggdb" )	
	
-- This AI should be last to compile
project "hybridAI"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../pprAI/include",
		"../sfAI/include",
		"../rvo2AI/include",
		"../footstepAI/include",
		"../ccAI/include",
		"../hybridAI/include",
		"../kdtree/include",
		"../external",
		"../util/include",
	}
	files { 
		"../hybridAI/include/*.h",
		"../hybridAI/src/*.cpp"
	}
	links { 
		"steerlib",
		"util",
		"pprAI",
		"rvo2dAI",
		"sfAI",
		"ccAI",
		"footstepAI",
		"kdtree"
	}
	targetdir "lib"	
	buildoptions("-std=c++0x -ggdb" )	
	
project "scenario"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../scenario/include",
		"../external",
		"../util/include",
	}
	files { 
		"../scenario/include/*.h",
		"../scenario/src/*.cpp"
	}
	links { 
		"steerlib",
		"util",
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	
project "kdtree"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../kdtree/include",
		"../external",
		"../util/include",
	}
	files { 
		"../kdtree/include/*.h",
		"../kdtree/src/*.cpp"
	}
	links { 
		"steerlib",
		"util",
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
                linkoptions {
                        "-install_name @rpath/libkdtree.dylib"
                }

project "Recast"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../navmeshBuilder/include",
		"../external/recastnavigation/Recast/Include",
		"../util/include",
	}
	files { 
		"../external/recastnavigation/Recast/Include/*.h",
		"../external/recastnavigation/Recast/Source/*.cpp",
	}
	links { 
		"steerlib",
		"util",
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
                linkoptions {
                        "-install_name @rpath/libRecast.dylib"
                }

project "DebugUtils"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../navmeshBuilder/include",
		"../external/recastnavigation/DebugUtils/Include",
		"../external/recastnavigation/Detour/Include",
		"../external/recastnavigation/Recast/Include",
		"../external/recastnavigation/DetourTileCache/Include",
		"../util/include",
	}
	files { 
		"../external/recastnavigation/DebugUtils/Include/*.h",
		"../external/recastnavigation/DebugUtils/Source/*.cpp",
	}
	links { 
		"Recast",
		"Detour"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
                linkoptions {
                        "-install_name @rpath/libDebugUtils.dylib"
                }

project "Detour"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../navmeshBuilder/include",
		"../external/recastnavigation/DebugUtils/Include",
		"../external/recastnavigation/Detour/Include",
		"../external/recastnavigation/Recast/Include",
		"../external/recastnavigation/DetourTileCache/Include",
		"../util/include",
	}
	files { 
		"../external/recastnavigation/Detour/Include/*.h",
		"../external/recastnavigation/Detour/Source/*.cpp",
	}
	links { 
		"Recast",
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
                linkoptions {
                        "-install_name @rpath/libDetour.dylib"
                }

project "DetourCrowd"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../navmeshBuilder/include",
		"../external/recastnavigation/DebugUtils/Include",
		"../external/recastnavigation/Detour/Include",
		"../external/recastnavigation/Recast/Include",
		"../external/recastnavigation/DetourTileCache/Include",
		"../external/recastnavigation/DetourCrowd/Include",
		"../util/include",
	}
	files { 
		"../external/recastnavigation/DetourCrowd/Include/*.h",
		"../external/recastnavigation/DetourCrowd/Source/*.cpp",
	}
	links { 
		"Recast",
		"Detour"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
                linkoptions {
                        "-install_name @rpath/libDetourCrowd.dylib"
                }	

project "navmesh"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../navmeshBuilder/include",
		"../external/recastnavigation/Recast/Include",
		"../external/recastnavigation/DebugUtils/Include",
		"../external/recastnavigation/Detour/Include",
		"../external/recastnavigation/DetourTileCache/Include",
		"../external/recastnavigation/DetourCrowd/Include",
		"../steersimlib/include",
		"../external",
		"../util/include",
		"../acclmesh/include"  -- for draw method
	}
	files { 
		"../navmeshBuilder/include/*.h",
		"../navmeshBuilder/src/*.cpp"
	}
	links { 
		"steerlib",
		"steersimlib",
		"util",
		"Recast",
		"DebugUtils",
		"Detour",
		"DetourCrowd",
		"acclmesh"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
                linkoptions {
                        "-install_name @rpath/libnavmesh.dylib"
                }

project "acclmesh"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../external",
		"../util/include",
		"../acclmesh/include"
	}
	files { 
		"../acclmesh/include/*.h",
		"../acclmesh/src/*.cpp"
	}
	links { 
		"steerlib",
		"util"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
                linkoptions {
                        "-install_name @rpath/libacclmesh.dylib"
                }
	
project "meshdatabase"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../external",
		"../util/include",
		"../acclmesh/include",
		"../meshdatabase/include",
		"../kdtree/include", 
	}
	files { 
		"../meshdatabase/include/*.h",
		"../meshdatabase/src/*.cpp"
	}
	links { 
		"steerlib",
		"util",
		"acclmesh",
		"kdtree"
	}
	targetdir "lib"
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
                linkoptions {
                        "-install_name @rpath/libmeshdatabase.dylib"
                }
	
project "steerbench"
	language "C++"
	kind "WindowedApp"
	includedirs { 
		"../steerlib/include",
		"../steerbench/include",
		"../external",
		"../util/include" 
	}
	files { 
		"../steerbench/include/*.h",
		"../steerbench/src/*.cpp"
	}
	links { 		
		"steerlib",
		"tinyxml",
		"util",
		"AntTweakBar",
		"glfw",
		"dl",
	}


	targetdir "bin"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		-- kind "ConsoleApp"
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`" 
		}
		linkoptions { 
			-- "-Wl,-rpath,./lib",
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}
		links { 		
			"GLU",
			"GL"
		}
		libdirs { "lib" }

	-- windows library cflags and libs
	configuration { "windows" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/x86" }
		links { 
			"opengl32",
			"glu32",
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		buildoptions { "-Wunused-value -Wshadow -Wreorder -Wsign-compare -Wall" }
		links { 
			"OpenGL.framework", 
			"Cocoa.framework",
		}

