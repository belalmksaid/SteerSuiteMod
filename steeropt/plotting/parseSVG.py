"""

    Dependencies:
    1. svg.path 
"""


from xml.dom import minidom
from svg.path import parse_path
import sys
import numpy as np


file = open(sys.argv[1], "r")
svg_string = file.read()
file.close()

scale = 0.5
_distance_threshold = 0.01 * scale
svg_dom = minidom.parseString(svg_string)

path_strings = [path.getAttribute('d') for path in svg_dom.getElementsByTagName('path')]
edges = []
verts = []

def distance(a, b):
    a=(a - b)
    d = np.sqrt((a*a).sum(axis=0))
    return d

def addVert(verts, point_):
    # check if vert allready in verts
    out=0
    distance_threshold = 0.0001
    for p in range(len(verts)):
        if (distance(verts[p], point_) < distance_threshold):
            return p
        
    verts.append(point_);
    return len(verts)-1
        
    
for path_string in path_strings:
    path_data = parse_path(path_string)
    # print path_data
    for i in range(len(path_data)):
        point = path_data[i]
        # point.start.real = X, point.start.imag =Y
        # print i, point.start.real, point.start.imag, point.start
        v0_ = np.array([point.start.real, point.start.imag]) * scale
        v1_ = np.array([point.end.real, point.end.imag]) * scale
        v0 = addVert(verts, v0_)
        v1 = addVert(verts, v1_)
        if ( distance(v0_, v1_) > _distance_threshold):
            edges.append([v0, v1])
        

    #  now use methods provided by the path_data object
    #  e.g. points can be extracted using 
    #  point = path_data.pos(pos_val) 
    #  where pos_val is anything between 0 and 1
    print "Number of verts: " + str(len(verts))
    
max_x=-10000000
min_x=10000000
max_z=-10000000
min_z=10000000
for vert in verts:
    if (vert[0] > max_x):
        max_x = vert[0]
    if (vert[1] > max_z):
        max_z = vert[1]
        
    if (vert[0] < min_x):
        min_x = vert[0]
    if (vert[1] < min_z):
        min_z = vert[1]    


delta_x = float(max_x + min_x) / 2.0
delta_z = float(max_z + min_z) / 2.0 
print max_x, min_x, max_z, min_z
graph_file = open("out.graph", "w")
for vert in verts:
    graph_file.write("v " + str(vert[0]-delta_x) + " 0 " + str(vert[1]-delta_z) + "\n" )
    
graph_file.write("\n")
edge_group_num=100
for e in range(len(edges)):
    if ((e == 0) or ((e % edge_group_num) == 0)):
        if (not graph_file.closed):
            graph_file.close()
        graph_file = open("out"+str(e)+".graph", "w")
        for vert in verts:
            graph_file.write("v " + str(vert[0]-delta_x) + " 0 " + str(vert[1]-delta_z) + "\n" )
            
        graph_file.write("\n")
    
    edge = edges[e]
    graph_file.write("e " + str(edge[0]) + " " + str(edge[1]) + "\n" )
    
graph_file.close()
    
