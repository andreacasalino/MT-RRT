**Multi Thread Rapidly Random exploring Trees**, aka **MT-RRT**, is a C++ library .
**Rapidly Random exploring Trees**, aka **RRT**, are a popular cathegory of robotic algorithms used for computing paths or trajectories in a constrained environment.
The most classical application is for serial manipulators path planning. This library implements multi-threaded strategies that are able to speed up
normal **RRt**, bidirectional **RRT** or **RRT***.

You can find detailed information in the [documentation](https://github.com/andreacasalino/MT-RRT/blob/master/doc/MT-RRT.pdf), which also provides a little background on **RRT**.
This library was conceived to solve any kind of problem. The only thing you need to do when solving a particular problem, is to derive a couple of objects that contain all the
problem-specif information. To clarify this process, 3 main classes of samples are reported, representative of 3 main classes of planning problems. follow the same approach
uased for the examples for your own custom problem.

**MT-RRT** is completely **cross platform**: let [CMake](https://cmake.org) do all the work for you.

**Dependencies**

**MT-RRT** internally makes use of [omp](https://en.wikipedia.org/wiki/OpenMP), which should be natively supported by any kind of modern compiler. 
The **omp** package is automatically searched by **CMake** when configuring the project. Just beware to use a compiler supporting it.

**Contents**

 * the documentation in ./doc explains both how to use **MT-RRT** as well give some theoretical background 
 * the core library is contained in ./MT-RRT
 * ./Samples contains some planning examples, extensively showing how to use **MT-RRT**. These should be intended as templates when deriving new custom problem.

**Compile**

 * Configure and generate the project using [CMake](https://cmake.org)
   * check this [tutorial](https://www.youtube.com/watch?v=LxHV-KNEG3k) (it is valid for any kind of IDE and not only VS) if you are not familiar
      
 * Compile the library and the samples
   * Some compiler (for example the **VisualStudio** one) might throw some warnings due to the fact that some classes inside **MT-RRT** are defined using the virtual inheritance. You can simply ingnore them.
   

# ![What you should see when running the application](https://github.com/andreacasalino/Easy-Factor-Graph-GUI/blob/master/Example.png)
