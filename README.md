<h1> Multi Threading RRT : MT_RRT </h1> 
C++ library containing multi threaded version of the RRT algorithm.

<h2> SOURCES</h2>

The sources of the library ar all contained in MT_RRT/Header and  MT_RRT/Source.
Samples stores some examples showing how to use this library.

<h2> HOW TO COMPILE </h2> 
<li> Windows: you can launch in Visual Studio Solution.sln or launch gpp_make.bat if you have installed g++ as compiler </li>
<li> Linux: use the Makefile </li> 

<h2> RUN THE EXAMPLES </h2>
After compiling, a folder called bin is created storing a static library and all the examples, together with the materials required to run them.
<li>  <font style="color:red"> bin/Sample_01_Points/ <font>: after running Sample_01_Points, several .html files will be created in Results/, which can be showed using your favourite browser </li>
<li> bin/Sample_02_Planar_Robots/01_Single_arm/:  after running 01_Single_arm, several .html files are created in Results/, which can be showed using your favourite browser </li>
<li> bin/Sample_02_Planar_Robots/02_Multiple_arms/:  after running 02_Multiple_arms, several .html files are created in Results/, which can be showed using your favourite browser </li>
<li> bin/Sample_02_Planar_Robots/00_GUI/:  run launcher.bat(.sh) in Windows(Linux) to have fun with a nice GUI, connected in back-end with this library. </li> 
<li> bin/03_Navigation/01_Navigation/:  after running 01_Navigation, several .html files are created in Results/, which can be showed using your favourite browser </li>
<li> bin/03_Navigation/01_Navigation_cluttered/:  after running 01_Navigation_cluttered, several .html files are created in Results/, which can be showed using your favourite browser </li>
<li> bin/03_Navigation/00_GUI/:  run launcher.bat(.sh) in Windows(Linux) to have fun with a nice GUI, connected in back-end with this library. </li> 

<h2> DOCUMENTATION </h2>

doc/documentation.pdf briefly gives you a minimal background about RRT and also documents the library sources. In particular, it explains also how to customize your own planning problem for using MT_RRT to solve it.
