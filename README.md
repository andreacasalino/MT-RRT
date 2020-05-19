# Multi Threading RRT (MT_RRT)
C++ library implementing parallelized implementations of the RRT algorithm.

CONTENTS:

./content/MT_RRT: contains the sources of the library
./content/Samples: contains some examples of planning problems. Have fun with the interactive GUIs that will be compiled in 
				   ./content/Samples/02_Planar_Robots/00_GUI and ./content/Samples/03_Navigation/00_GUI
./content/EFG.pdf: an extensive guide that contains details about the theoretical concepts MT_RRT is based on as well the structure of the classes constituting MT_RRT.
					It explains also how to customize your own planning problem for using MT_RRT to solve it.

INSTALLATION(*):

->Windows: Use the Visual Studio solution ./content/Solution.sln to compile MT_RRT
		   Alternatively you can build the library and all the examples using g++, by launching gpp_make.bat

->UNIX: Use ./content/Makefile to compile MT_RRT as well as all the examples.

* In all cases, a folder at the location ./content/bin is created, storing all the binaries as well as the files required to run them
