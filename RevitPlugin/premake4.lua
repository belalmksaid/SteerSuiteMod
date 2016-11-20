
--[====[
project "RevitAdapter"
	-- location "build/CSharpApp"
	-- libdirs { "ManagedDll" }
	language "C#"
	kind "SharedLib"
	files {
		-- "RevitAdapter/*.resx",
		-- "RevitAdapter/*.settings", 
		"RevitAdapter/*.cs" }
	links {
		"System",
		"System.Data",
		"System.Deployment",
		"System.Drawing",
		"System.Windows.Forms",
		"System.Xml"
	}
	
	configuration "Debug"
		defines { "DEBUG" }
		flags { "Symbols" }
	
	configuration "Release"
		defines { "NDEBUG" }
		flags { "Optimize" }
--]====]
	
project "RevitPlugin"
	-- location "build/CSharpApp"
	-- libdirs { "ManagedDll" }
	kind "SharedLib"
	language "C#"
	files {
		-- "RevitPlugin/*.resx",
		-- "RevitPlugin/*.settings", 
		"../RevitPlugin/RevitPlugin/*.cs" 
	}
	links {
		"System",
		"System.Data",
		"System.Deployment",
		"System.Drawing",
		"System.Windows",
		"System.Windows.Forms",
		"System.Xml",
		"steerSuiteAdapterCSharp",
		"PresentationFramework",
	}
	
	configuration "Debug"
		defines { "DEBUG" }
		flags { "Symbols" }
	
	configuration "Release"
		defines { "NDEBUG" }
		flags { "Optimize" }

--[====[
project "SteerSuiteAdapterTester"
	-- location "build/CSharpApp"
	-- libdirs { "ManagedDll" }
	kind "ConsoleApp"
	language "C#"
	files {
		"SteerSuiteAdapterTester/*.resx",
		"SteerSuiteAdapterTester/*.settings", 
		"SteerSuiteAdapterTester/*.cs" }
	links {
		"System",
		"System.Data",
		"System.Deployment",
		"System.Drawing",
		"System.Windows.Forms",
		"System.Xml",
		"RevitAdapter"
	}
	
	configuration "Debug"
		defines { "DEBUG" }
		flags { "Symbols" }
	
	configuration "Release"
		defines { "NDEBUG" }
		flags { "Optimize" }
--]====]