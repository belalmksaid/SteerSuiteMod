A node on the upper floor

../build/bin/steersim -testcase ../acclmesh/data/blank.xml -config ../acclmesh/data/ACCLMesh-config.xml -numFrames 1000 -module acclmesh,load_obj=../acclmesh/data/staircase2.obj,targetNode=1500

For debug
build/bin/steersim -ai rvo2dAI -testcase acclmesh/data/underpass.xml -config acclmesh/data/ACCLMesh_underpass-config.xml -numFrames 1000 -module acclmesh,load_obj=acclmesh/data/overhang3.obj