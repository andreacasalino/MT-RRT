This folder contains 3 examples of usage of **MT-RRT**.

It is strongly recommended to start from [ReadMeExample](../ReadMeExample.cpp), which is not meant to be compilable, as it shows how to use in a nutshell **MT-RRT**.

The 3 examples are stored in:

- **problem-trivial**, which is the kind of problem described in the [documentation](../doc/MT-RRT.pdf) at Section "Planar maze problem"

- **problem-planarRobots**, which is the kind of problem described in the [documentation](../doc/MT-RRT.pdf) at Section "Articulated arm problem"

- **problem-navigation**, which is the kind of problem described in the [documentation](../doc/MT-RRT.pdf) at Section "Navigation problem"

Each class of example contains the sources implementing the specific conenctor and sampler, under header and src, while folder named samples contains some sample scenarios. Each scenario is a **json** describing the scenario itself with the following contents:
- **parameters**, used to describe the **mt_rrt::Parameters** to adopt with the obvious meaning of notation
- **planner** with:
    - **type**, specifiying the kind of planner to use. Value for this field can be any of:
        - STANDARD -> classic mono-thread RRT
        - EMBARASS -> to use the EmbarassinglyParallelPlanner
        - PQUERY -> to use the ParallelizedQueriesPlanner
        - SHARED -> to use the SharedTreePlanner
        - LINKED -> to use the LinkedTreesPlanner
        - MULTIAG > to use the MultiAgentPlanner
    - **threads**, specifying the number of threads that can be used by the planner (it is discarded in case type is STANDARD)
    - **synchronization**, for specyfing the synchronization degree in case the type of planner is LINKED or MULTIAG
- **scene**, which contains problem specific information describing the problem
- an array of **cases**, which are the problems to solve for that scenario. **parameters** can be overriden at the single case level. For example you can specify to use more threads for only one case and not the others.

How to run the samples?
- If you use **Visual Studio** in **Windows**, the samples will be extra target, one per **json**,  that can be compiled and runned. After all results will be computed, they will be saved into a special folder and later accessed by a python script that will render them. The python script will be automatically runned after results will be ready, in order to render them.
- In all other cases, samples will be created as **CMake** [custom targets](https://cmake.org/cmake/help/latest/command/add_custom_target.html), which are basically post build command that can be run from the command line. Therefore, any sample can be run in the same way any other target is build. For example using ninja in Linux, go to the build folder and run `ninja MT-RRT-problem-planarRobots-Samples-Scenario-04` (or any other sample). Results will be computed and rendered using a python script as similarly described for **Visual Studio**.

Feel free to play with the **json** files, change the parameters or the kind of planner and see the results!

