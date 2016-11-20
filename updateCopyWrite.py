# updates the copyright information for all .cs files
# usage: call recursive_traversal, with the following parameters
# parent directory, old copyright text content, new copyright text content

#
# Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
# See license.txt for complete license.
#

# Added support for filetypes that use a different kind of source file.
# uses #s for comments instead of //s for example

import os
import sys
import re


excludedir = ["..\\Lib"]

file_match='\.cpp$|\.h$|\.hpp$'
file_match_bash='Makefile$'
file_match_XML='.xml$'


def update_source(filename, oldcopyright, copyright):
    utfstr = chr(0xef)+chr(0xbb)+chr(0xbf)
    fdata = file(filename,"r+").read()
    isUTF = False
    if (fdata.startswith(utfstr)):
        isUTF = True
        fdata = fdata[3:]
    if (oldcopyright != None):
        if (fdata.find(oldcopyright) > -1):
            fdata = fdata.replace( oldcopyright, copyright)
            print "Updating copywrite for " + filename
        elif (fdata.find(copyright) > -1):
            print "CopyWrite up to date for " + filename
            return
        else:
            print "adding copywrite to " + filename
            fdata = copyright + fdata
    if (isUTF):
        file(filename,"w").write(utfstr+fdata)
    else:
        file(filename,"w").write(fdata)

def recursive_traversal(dir,  oldcopyright, copyright, matchy):
    global excludedir
    fns = os.listdir(dir)
    # print "listing "+dir
    for fn in fns:
        fullfn = os.path.join(dir,fn)
        if (fullfn in excludedir):
            continue
        if (os.path.isdir(fullfn)):
            recursive_traversal(fullfn, oldcopyright, copyright, matchy)
        else:
            if (re.search(matchy, fullfn)):
                update_source(fullfn, oldcopyright, copyright)

if len(sys.argv) != 2:
	print "Incoreect number of arguments"
	print "Usage: python updateCopyWrite.py src/"
	sys.exit(-1)

oldcright = file("copywrite/oldcr.txt","r+").read()
cright = file("copywrite/copyrightText.txt","r+").read()
directory = filename = sys.argv[1]
print "Old copywrite: " 
print oldcright
print ""
print "New CopyWrite: " 
print cright
recursive_traversal(directory, oldcright, cright, file_match)


oldcright = file("copywrite/oldcr_bash.txt","r+").read()
cright = file("copywrite/newBashCopyWrite.txt","r+").read()
directory = filename = sys.argv[1]
print "Old copywrite: " 
print oldcright
print ""
print "New CopyWrite: " 
print cright
recursive_traversal(directory, oldcright, cright, file_match_bash)

oldcright = file("copywrite/oldxmlcopywrite.txt","r+").read()
cright = file("copywrite/newxmlcopywrite.txt","r+").read()
directory = filename = sys.argv[1]
print "Old copywrite: " 
print oldcright
print ""
print "New CopyWrite: " 
print cright
recursive_traversal(directory, oldcright, cright, file_match_XML)

exit()
