

cuda_enable=0
cuda_npp_enable=0
if not os.getenv("CUDA_INC_PATH") then
	printf("CUDA_INC_PATH: environement variable not found.")
else
	cuda_enable=cuda_enable+1
	printf("CUDA_INC_PATH: OK.")
end
if not os.getenv("CUDA_BIN_PATH") then
	printf("CUDA_BIN_PATH: environement variable not found.")
else
	cuda_enable=cuda_enable+1
	printf("CUDA_BIN_PATH: OK.")
end
if not os.getenv("CUDA_LIB_PATH") then
	printf("CUDA_LIB_PATH: environement variable not found.")
else
	cuda_enable=cuda_enable+1
	printf("CUDA_LIB_PATH: OK.")
end
if not os.getenv("CUDA_LIB64_PATH") then
	printf("Warning: CUDA_LIB64_PATH: environement variable not found. Please set it on X64 system.")
else
	printf("CUDA_LIB64_PATH: OK.")
end
if not os.getenv("NVSDKCOMPUTE_ROOT") then
	printf("NVSDKCOMPUTE_ROOT: environement variable not found..")
else
	cuda_enable=cuda_enable+1
	printf("NVSDKCOMPUTE_ROOT: OK.")
end
if not os.getenv("NPP_SDK_PATH") then
	printf("NPP_SDK_PATH: environement variable not found, disabling NPP plugin.")
else
	cuda_npp_enable=1
	printf("NPP_SDK_PATH: OK.")
end
if not os.getenv("NPP_SDK_PATH") then
	printf("NPP_SDK_PATH: environement variable not found, disabling NPP plugin.")
else
	cuda_npp_enable=1
	printf("NPP_SDK_PATH: OK.")
end
if not (cuda_enable>=3)then
	cuda_enable=0
	printf "WARNING:\t NVIDIA CUDA toolkit not setup on this computer. GpuCV CUDA plugin disabled."
end

project "spacesyntax"
	language "C++"
	kind "SharedLib"

	includedirs { 
		"../steerlib/include",
		"../spacesyntax/include",
		"../external",
		"../external/Eigen3.2.7",
		"../util/include",
	}
	files { 
		"../spacesyntax/include/*.h",
		"../spacesyntax/src/*.cpp",
		"../spacesyntax/src/*.cu"
	}
	links { 
		"steerlib",
		"util"
	}

	-- targetdir "bin"
	buildoptions("-std=c++0x -ggdb" )	

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",
			"-fopenmp",
--			"-lgomp"
		}
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`",
			"-fopenmp",
		}
		-- libdirs { "lib" }
		links {
			"X11",
			"tinyxml",
			"dl",
			"pthread"
		}

	-- windows library cflags and libs
	configuration { "windows" }
		-- libdirs { "lib" }
		buildoptions { "/openmp" }
		if cuda_enable > 0 then
			links { 
				"opengl32",
				"glu32",
				"cudart"
			}
		else
			printf "WARNING:\t GpuCV CUDA plugin disabled."
			printf "WARNING:\t GpuCV CUDA plugin disabled."
			printf "WARNING:\t GpuCV CUDA plugin disabled."
			links { 
				"opengl32",
				"glu32",
			}
		end

	-- mac includes and libs
	configuration { "macosx" }
		-- kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		-- includedirs { "/Library/Frameworks/SDL.framework/Headers" }
		buildoptions { "-std=c++0x -ggdb -fPIC -fopenmp" }
		linkoptions { 
			"-Wl,-rpath," .. path.getabsolute("lib") ,
                        "-install_name @rpath/libspacesyntax.dylib"
		}
		-- libdirs { "lib" }
		links { 
			-- "OpenGL.framework", 
			-- "Cocoa.framework",
			-- "tinyxml",
			"dl",
			"pthread"
		}
