# MT_RRT
General purpose library for multithreaded Rapidly Random exploring Trees

For any problems contact andrecasa91@gmail.com

All the functionalities are contained in MT_RRT:

-MT_RRT/Header contains all the header files
-MT_RRT/Lib contains the compiled libraries: MT_RRT.lib for Release version; MT_RRTd.lib for Debug


The Visual Studio 2015 project MT_RRT.sln contains five projects:

-MT_RRT/MT_RRT; which contains a Basic usage example (Basic_sample.cpp) , showing the functionalities of the library 
    For this example only the serial standard RRT planners are considered
-MT_RRT/Problem_01,Problem_02,Problem_03,Problem_04 are examples solved by making use of the multithreading
	strategies available in the library.
The example provided read files in relative folder: always launch the exe from their folder location.	
In Debug mode, the planner when expanding the search tree(s) display also the iterations reached.


	
For creating a planning problem (see the Basic_sample.cpp), you must istantiate class Planner_basic, with the following syntax:
Planner_basic new_problem(Qo, Qf, factory, deterministic_coeff, max_iterations, expansion_mode, strategy_handler)

where:

-Qo, is a collection of vectors describing the initial state. Usually a single vector is sufficient for defining the state, use the list format for those cases when you want 
           to separate different quantities (for instance consider the list <position, velocity>).
-Qf, is a collection of vectors describing the final state. A solution for the problem is a path connecting Qo to Qf
-factory, is a Node factory: it is a class describing your particular planning problem (see also the following). You can derive your own factory, for describing a problem different 
          from the one proposed 
-deterministic_coeff, is the percentage of times the planner tries to connect Qf directly to the searching without sampling a random state. Use a low value for this parameter,
          to not compromise the probabilistic completeness of the planner
-expansion_mode, is the expansion strategy (single tree, bidirectional or RRT*)
-strategy_handler, is a class explaining the planner which strategy you want to use for parallelizing the search as well the number of threads you want to use. You can use the static method Planner_basic::Get_Handler to simply select the strategy exploiting predefined ennums (see the examples provided in Problem_01.cpp, Problem_02.cpp, Problem_03.cpp and Problem_04.cpp)



The definition of the Node factory to use is central for defining the problem you want to solve. Basically I_Node_Factory is an empty interface to be derived. In the derived 
factories, you must override a series of virtual methods which explain the planner how to extend the tree for your problem: which is the optimal trajectory you consider; how is
defined the admittance check for new state; how to randomly sample new state for the system and others. Follow the directions provided in Node.h and 
the implementation of Path_simple_Node_Factory contained in Node.cpp 

Path_simple_Node_Factory in Node.h, is a factory describing a standard path planning problem, embedded with some basic functionalties for collisions check. 
You can derive much more sophisticated factories for exploiting more complete physics simulators, with their own collision check modules. 
