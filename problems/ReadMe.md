This folder contains 3 examples of usage of **MT-RRT**.

It is strongly recommended to start from *ReadMe.cpp*, which is not meant to be compilable, but
simply shows how to use in a nutshell **MT-RRT**.

The 3 examples are stored in:

- TrivialProblems, which is the kind of problem described in the [documentation](../doc/MT-RRT.pdf) at Section "Planar maze problem"

- PlanarRobotsProblems, which is the kind of problem described in the [documentation](../doc/MT-RRT.pdf) at Section "Articulated arm problem"

- NavigationProblems, which is the kind of problem described in the [documentation](../doc/MT-RRT.pdf) at Section "Navigation problem"

Each sample target configures a planning problem by reading some information from a default .json file, whose fields can be partially/totally overridden
by a user defined .json (check documentation of mt_rrt::utils::SampleFramework for the details).
The purpose is to solve an array of specific problems and log the results into some generated .json files, whose information can be displayed by running
specific python scripts. The python scipts to run are displayed into the console application by each specific sample.

In all samples, except for *ReadMe.cpp*, most of the computations will be done behind the scenes and that's why 
yout should start from *ReadMe.cpp* to first understand in essence how **MT-RRT** works.
