This folder contains 3 examples of usage of **MT-RRT**.

It is strongly recommended to start from *ReadMe.cpp*, which is not meant to be compilable, but
simply shows how to use in a nutshell **MT-RRT**.

The 3 examples are stored in:
	- TrivialProblems, which is the kind of problem described in the documentation at Section "Planar maze problem"
	- PlanarRobotsProblems, which is the kind of problem described in the documentation at Section "Articulated arm problem"
	- NavigationProblems, which is the kind of problem described in the documentation at Section "Navigation problem"

In each of the above samples **MT-RRT** is used to solve a particular planning problem using some default parameters.
You can override such parameters if you want, by passing some arguments to the samplea application (refer to the 
ReadMe.md inside each sample folder).

In all samples, except for *ReadMe.cpp*, most of the computations will be done behind the scenes and that's why 
yout should start from *ReadMe.cpp* to first understand in essence how **MT-RRT** works.
