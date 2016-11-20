
-- Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
-- See license.txt for complete license

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
	
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
		links {
			"OpenGL.framework",
                }
	
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
	
	buildoptions("-std=c++0x -ggdb" )	
	
-- This AI should be last steering algorithm to compile
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
	
	buildoptions("-std=c++0x -ggdb" )	

project "acclmesh"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../steerlib/include",
		"../external",
		"../external/GTEngine/Include",
		"../util/include",
		"../acclmesh/include",
		"../navmeshBuilder/include"
	}
	files { 
		"../acclmesh/include/*.h",
		"../acclmesh/src/*.cpp"
	}
	links { 
		"steerlib",
		"util",
		"navmesh"
	}
	
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
		"../navmeshBuilder/include",
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
		"kdtree",
		"navmesh"
	}
	
	buildoptions("-std=c++0x -ggdb" )	
	configuration { "macosx" }
                linkoptions {
                        "-install_name @rpath/libmeshdatabase.dylib"
                }

--[====[	
project "AntTweakBar"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../external/AntTweakBar/include",
		"../external/AntTweakBar/src",
	}
	files { 
		"../external/AntTweakBar/src/*.h",
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
		"../external/AntTweakBar/src/TwEventX11.c"
	}
	targetdir "lib"		
	
	configuration { "linux", "gmake" }
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
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
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
--]====]	               
                
 include "../steerdemo"
 include "../external/libcmaes"
 include "../steeropt"
 include "../spacesyntax"
 include "../vgAI"
 include "../steerplugin"


-- Put you new project file before these!!!
 include "../steerpluginTester"
 include "../RevitPlugin"
 include "../steerSuiteAdapter"
