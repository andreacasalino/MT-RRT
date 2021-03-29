**Multi Thread Rapidly Random exploring Trees**, aka **MT-RRT**, is a C++ library that can be used for path or trajectory planning.
**Rapidly Random exploring Trees**, aka **RRT**, are a popular cathegory of robotic algorithms used for computing paths or trajectories in a constrained environment.
This library implements multi-threaded strategies that are able to speed up
normal **RRT**, **bidirectional  RRT** or **RRT***.

You can find detailed information in the [documentation](https://github.com/andreacasalino/MT-RRT/blob/master/doc/MT-RRT.pdf), which also provides a little background on **RRT**.
This library was conceived to solve any kind of problem. The only thing you need to do when solving a new kind of problem, is to derive a couple of objects that contain all the
problem-specific information. To clarify this process, 3 main classes of samples are reported, representative of 3 main classes of planning problems. Follow the same approach
used for the examples for your own custom problem.

**MT-RRT** is completely **cross platform**: let [CMake](https://cmake.org) do all the work for you.

**Dependencies**

**MT-RRT** internally makes use of [omp](https://en.wikipedia.org/wiki/OpenMP), which should be natively supported by any kind of modern compiler. 
The **omp** package is automatically searched by **CMake** when configuring the project. Just beware to use a compiler supporting it.
It can be sometimes usefull to check the progress of the solver, printing on the console the iterations done so far. This is not the default behaviour of **MT-RRT** but can be enabled
turning ON the CMake option called SHOW_PROGRESS_OPT.

**Contents**

 * the documentation in ./doc explains both how to use **MT-RRT** as well give some theoretical background 
 * the core library is contained in ./MT-RRT
 * samples are contained in ./Samples. They extensively shows how to use **MT-RRT**. They should be intended as templates when deriving new custom problem.
	* after running any of the sample, a .json file storing ther results will be created. You can later use a python script copied in the same folder to visualize such results. In particular, you will see the trajectory/path computed as solution in a form of animation and some of the states pertaining to the obtained exploring trees (blue for the first one, green for the second in case a **bidirectional  RRT** was used)

**Compile**

 * Configure and generate the project using [CMake](https://cmake.org)
   * check this [tutorial](https://www.youtube.com/watch?v=LxHV-KNEG3k) (it is valid for any kind of IDE and not only VS) if you are not familiar
      
 * Compile the library and the samples
   * Some compiler (for example the **VisualStudio** one) might throw some warnings due to the fact that some classes inside **MT-RRT** are defined using the virtual inheritance. You can simply ingnore them.
  
 **Star**

If you have found this library useful, please find the time to star it :)
