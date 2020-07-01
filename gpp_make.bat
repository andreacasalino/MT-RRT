REM change the following path to match the directory where MinGW is installed on your machine
SET MINGW_PATH=C:/MinGW/lib/gcc/mingw32/6.3.0/
SET OPT=O3


REM --------> build the library <----------
RD ".\bin" /S /Q
MD ".\bin"
MD ".\bin\src_JS"
XCOPY  /S ".\Samples\src_JS" ".\bin\src_JS"
MD ".\bin\src_Py"
XCOPY  /S ".\Samples\src_Py" ".\bin\src_Py"

g++ -c ./MT_RRT/Source/json.cpp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Problem_description.cpp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Tree.cpp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Extensions.cpp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Simplifier.cpp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Planner.cpp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Planner_canonical.cpp -%OPT% -Wall -DDETER_SEED
g++ -c ./MT_RRT/Source/Planner_MT.cpp  -fopenmp -%OPT% -Wall -DDETER_SEED
g++ -c ./MT_RRT/Source/Planner_query_parall.cpp  -fopenmp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Planner_shared_parall.cpp  -fopenmp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Planner_copied_parall.cpp  -fopenmp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Planner_multi_agents.cpp  -fopenmp -%OPT% -Wall
g++ -c ./MT_RRT/Source/Problem_path_basic.cpp -%OPT% -Wall

ar rvs ./bin/MT_RRT.lib ^
	json.o ^
	Problem_description.o ^
	Tree.o ^
	Extensions.o ^
	Simplifier.o ^
	Planner.o ^
	Planner_canonical.o ^
	Planner_MT.o ^
	Planner_query_parall.o ^
	Planner_shared_parall.o ^
	Planner_copied_parall.o ^
	Planner_multi_agents.o ^
	Problem_path_basic.o
del *.o

	
REM --------> build samples <----------

REM --------> build Points example <----------
	MD ".\bin\Sample_01_Points"
	MD ".\bin\Sample_01_Points\Results"
	COPY  ".\Samples\01_Points\print_result.js" ".\bin\Sample_01_Points\Results\print_result.js"
	g++ -o ./bin/Sample_01_Points/Sample_01_Points ./Samples/01_Points/Main.cpp  ./Samples/01_Points/src/Problem_points.cpp  ./bin/MT_RRT.lib %MINGW_PATH%libgomp.a -pthread -%OPT% -Wall

REM --------> build Planar arm example, single robot <----------
	MD ".\bin\Sample_02_Planar_Robots\01_Single_arm"
	COPY ".\Samples\02_Planar_Robots\01_Single_arm\problem.json" ".\bin\Sample_02_Planar_Robots\01_Single_arm\problem.json"
	MD ".\bin\Sample_02_Planar_Robots\01_Single_arm\Results"
	COPY ".\Samples\02_Planar_Robots\print_result.js" ".\bin\Sample_02_Planar_Robots\01_Single_arm\Results\print_result.js"
	COPY ".\Samples\02_Planar_Robots\00_GUI\front_JS\src\Robot.js" ".\bin\Sample_02_Planar_Robots\01_Single_arm\Results\Robot.js"
	g++ -o ./bin/Sample_02_Planar_Robots/01_Single_arm/01_Single_arm ./Samples/02_Planar_Robots/01_Single_arm/Main.cpp ./Samples/src/geometry.cpp ./Samples/02_Planar_Robots/src/Problem_planar_arms.cpp ./bin/MT_RRT.lib %MINGW_PATH%libgomp.a -pthread -%OPT% -Wall

REM --------> build Planar arm example, multiple robots <----------
	MD ".\bin\Sample_02_Planar_Robots\02_Multiple_arms"
	COPY ".\Samples\02_Planar_Robots\02_Multiple_arms\problem.json" ".\bin\Sample_02_Planar_Robots\02_Multiple_arms\problem.json"
	MD ".\bin\Sample_02_Planar_Robots\02_Multiple_arms\Results"
	COPY ".\Samples\02_Planar_Robots\print_result.js" ".\bin\Sample_02_Planar_Robots\02_Multiple_arms\Results\print_result.js"
	COPY ".\Samples\02_Planar_Robots\00_GUI\front_JS\src\Robot.js" ".\bin\Sample_02_Planar_Robots\02_Multiple_arms\Results\Robot.js"
	g++ -o ./bin/Sample_02_Planar_Robots/02_Multiple_arms/02_Multiple_arms ./Samples/02_Planar_Robots/02_Multiple_arms/Main.cpp ./Samples/src/geometry.cpp ./Samples/02_Planar_Robots/src/Problem_planar_arms.cpp ./bin/MT_RRT.lib %MINGW_PATH%libgomp.a -pthread -%OPT% -Wall

REM --------> build Planar arm example, interactive GUI <----------
	MD ".\bin\Sample_02_Planar_Robots\00_GUI"
	MD ".\bin\Sample_02_Planar_Robots\00_GUI\front_JS"
	XCOPY  /S ".\Samples\02_Planar_Robots\00_GUI\front_JS" ".\bin\Sample_02_Planar_Robots\00_GUI\front_JS"
	COPY  ".\Samples\02_Planar_Robots\00_GUI\launcher.bat" ".\bin\Sample_02_Planar_Robots\00_GUI\launcher.bat"
	g++ -o ./bin/Sample_02_Planar_Robots/00_GUI/back_Cpp ./Samples/02_Planar_Robots/00_GUI/back_Cpp/Main.cpp ./Samples/src/geometry.cpp ./Samples/02_Planar_Robots/src/Problem_planar_arms.cpp ./Samples/src/Stream_Socket.cpp ./bin/MT_RRT.lib -DMINGW_COMPILE %MINGW_PATH%libgomp.a -pthread -lws2_32 -%OPT%

REM --------> build Navigation problem <----------
	MD ".\bin\Sample_03_Navigation\01_Navigation"
	COPY ".\Samples\03_Navigation\01_Navigation\problem.json" ".\bin\Sample_03_Navigation\01_Navigation\problem.json"
	MD ".\bin\Sample_03_Navigation\01_Navigation\Results"
	COPY ".\Samples\03_Navigation\print_result.js" ".\bin\Sample_03_Navigation\01_Navigation\Results\print_result.js"
	g++ -o ./bin/Sample_03_Navigation/01_Navigation/01_Navigation ./Samples/03_Navigation/01_Navigation/Main.cpp ./Samples/src/geometry.cpp ./Samples/03_Navigation/src/Problem_Navigation.cpp ./bin/MT_RRT.lib %MINGW_PATH%libgomp.a -pthread -%OPT% -Wall

REM --------> build Navigation problem cluttered <----------
	MD ".\bin\Sample_03_Navigation\02_Navigation_cluttered"
	COPY ".\Samples\03_Navigation\02_Navigation_cluttered\problem.json" ".\bin\Sample_03_Navigation\02_Navigation_cluttered\problem.json"
	MD ".\bin\Sample_03_Navigation\02_Navigation_cluttered\Results"
	COPY ".\Samples\03_Navigation\print_result.js" ".\bin\Sample_03_Navigation\02_Navigation_cluttered\Results\print_result.js"
	g++ -o ./bin/Sample_03_Navigation/02_Navigation_cluttered/02_Navigation_cluttered ./Samples/03_Navigation/02_Navigation_cluttered/Main.cpp ./Samples/src/geometry.cpp ./Samples/03_Navigation/src/Problem_Navigation.cpp ./bin/MT_RRT.lib %MINGW_PATH%libgomp.a -pthread -%OPT% -Wall

REM --------> build Navigation example, interactive GUI <----------
	MD ".\bin\Sample_03_Navigation\00_GUI"
	MD ".\bin\Sample_03_Navigation\00_GUI\front_JS"
	XCOPY  /S ".\Samples\03_Navigation\00_GUI\front_JS" ".\bin\Sample_03_Navigation\00_GUI\front_JS"
	COPY  ".\Samples\03_Navigation\00_GUI\launcher.bat" ".\bin\Sample_03_Navigation\00_GUI\launcher.bat"
	g++ -o ./bin/Sample_03_Navigation/00_GUI/back_Cpp ./Samples/03_Navigation/00_GUI/back_Cpp/Main.cpp ./Samples/src/geometry.cpp ./Samples/03_Navigation/src/Problem_Navigation.cpp ./Samples/src/Stream_Socket.cpp ./bin/MT_RRT.lib -DMINGW_COMPILE %MINGW_PATH%libgomp.a -pthread -lws2_32 -%OPT%
	
	

