all: build_lib build_samples

build_lib:
	mkdir -p bin
	mkdir -p bin/src_JS
	cp ./Samples/src_JS ./bin/src_JS -r
	mkdir -p bin/src_Py
	cp ./Samples/src_Py ./bin/src_Py -r
	g++ -c ./MT_RRT/Source/json.cpp -O3 -Wall
	g++ -c ./MT_RRT/Source/Problem_description.cpp -O3 -Wall
	g++ -c ./MT_RRT/Source/Tree.cpp -O3 -Wall -D_REW_DEBUG
	g++ -c ./MT_RRT/Source/Extensions.cpp -O3 -Wall
	g++ -c ./MT_RRT/Source/Planner.cpp -O3 -Wall
	g++ -c ./MT_RRT/Source/Planner_canonical.cpp -O3 -Wall -DDETER_SEED
	g++ -c ./MT_RRT/Source/Planner_MT.cpp  -fopenmp -O3 -Wall -DDETER_SEED
	g++ -c ./MT_RRT/Source/Planner_query_parall.cpp  -fopenmp -O3 -Wall
	g++ -c ./MT_RRT/Source/Planner_shared_parall.cpp  -fopenmp -O3 -Wall
	g++ -c ./MT_RRT/Source/Planner_copied_parall.cpp  -fopenmp -O3 -Wall
	g++ -c ./MT_RRT/Source/Planner_multi_agents.cpp  -fopenmp -O3 -Wall
	g++ -c ./MT_RRT/Source/Problem_path_basic.cpp -O3 -Wall
	ar rvs ./bin/MT_RRT.a \
		json.o \
		Problem_description.o \
		Tree.o \
		Extensions.o \
		Planner.o \
		Planner_canonical.o \
		Planner_MT.o \
		Planner_query_parall.o \
		Planner_shared_parall.o \
		Planner_copied_parall.o \
		Planner_multi_agents.o \
		Problem_path_basic.o
	rm *.o
	
build_samples: Sample_Points Sample_Planar_single Sample_Planar_multi Sample_Planar_GUI Sample_Navigation Sample_Navigation_cluttered Sample_Navigation_GUI

Sample_Points:
	mkdir -p ./bin/Sample_01_Points
	mkdir -p ./bin/Sample_01_Points/Results
	cp  ./Samples/01_Points/print_result.js ./bin/Sample_01_Points/Results/print_result.js -r
	g++ -o ./bin/Sample_01_Points/Sample_01_Points ./Samples/01_Points/Main.cpp  ./Samples/01_Points/src/Problem_points.cpp  ./bin/MT_RRT.a -fopenmp -pthread -O3 -Wall
	
Sample_Planar_single:
	mkdir -p ./bin/Sample_02_Planar_Robots/01_Single_arm
	cp ./Samples/02_Planar_Robots/01_Single_arm/problem.json ./bin/Sample_02_Planar_Robots/01_Single_arm/problem.json -r
	mkdir -p ./bin/Sample_02_Planar_Robots/01_Single_arm/Results
	cp ./Samples/02_Planar_Robots/print_result.js ./bin/Sample_02_Planar_Robots/01_Single_arm/Results/print_result.js -r
	cp ./Samples/02_Planar_Robots/00_GUI/front_JS/src/Robot.js ./bin/Sample_02_Planar_Robots/01_Single_arm/Results/Robot.js -r
	g++ -o ./bin/Sample_02_Planar_Robots/01_Single_arm/01_Single_arm ./Samples/02_Planar_Robots/01_Single_arm/Main.cpp ./Samples/src/geometry.cpp ./Samples/02_Planar_Robots/src/Problem_planar_arms.cpp ./bin/MT_RRT.lib -fopenmp -pthread -O3 -Wall
	
Sample_Planar_multi:
	mkdir -p ./bin/Sample_02_Planar_Robots/02_Multiple_arms
	cp ./Samples/02_Planar_Robots/02_Multiple_arms/problem.json ./bin/Sample_02_Planar_Robots/02_Multiple_arms/problem.json -r
	mkdir -p ./bin/Sample_02_Planar_Robots/02_Multiple_arms/Results
	cp ./Samples/02_Planar_Robots/print_result.js ./bin/Sample_02_Planar_Robots/02_Multiple_arms/Results/print_result.js -r
	cp ./Samples/02_Planar_Robots/00_GUI/front_JS/src/Robot.js ./bin/Sample_02_Planar_Robots/02_Multiple_arms/Results/Robot.js -r
	g++ -o ./bin/Sample_02_Planar_Robots/02_Multiple_arms/02_Multiple_arms ./Samples/02_Planar_Robots/02_Multiple_arms/Main.cpp ./Samples/src/geometry.cpp ./Samples/02_Planar_Robots/src/Problem_planar_arms.cpp ./bin/MT_RRT.lib -fopenmp -pthread -O3 -Wall
	
Sample_Planar_GUI:
	mkdir -p ./bin/Sample_02_Planar_Robots/00_GUI
	mkdir -p ./bin/Sample_02_Planar_Robots/00_GUI/front_JS
	cp  ./Samples/02_Planar_Robots/00_GUI/front_JS ./bin/Sample_02_Planar_Robots/00_GUI/front_JS -r
	cp ./Samples/02_Planar_Robots/00_GUI/launcher.bat ./bin/Sample_02_Planar_Robots/00_GUI/launcher.bat -r
	g++ -o ./bin/Sample_02_Planar_Robots/00_GUI/back_Cpp ./Samples/02_Planar_Robots/00_GUI/back_Cpp/Main.cpp ./Samples/src/geometry.cpp ./Samples/02_Planar_Robots/src/Problem_planar_arms.cpp ./Samples/src/Stream_Socket.cpp ./bin/MT_RRT.lib -fopenmp -pthread -lws2_32 -O3

Sample_Navigation:
	mkdir -p ./bin/Sample_03_Navigation/01_Navigation
	cp ./Samples/03_Navigation/01_Navigation/problem.json ./bin/Sample_03_Navigation/01_Navigation/problem.json -r
	mkdir -p ./bin/Sample_03_Navigation/01_Navigation/Results
	cp ./Samples/03_Navigation/print_result.js ./bin/Sample_03_Navigation/01_Navigation/Results/print_result.js -r
	g++ -o ./bin/Sample_03_Navigation/01_Navigation/01_Navigation ./Samples/03_Navigation/01_Navigation/Main.cpp ./Samples/src/geometry.cpp ./Samples/03_Navigation/src/Problem_Navigation.cpp ./bin/MT_RRT.lib -fopenmp -pthread -O3 -Wall

Sample_Navigation_cluttered:
	mkdir -p .\bin\Sample_03_Navigation\02_Navigation_cluttered -r
	cp .\Samples\03_Navigation\02_Navigation_cluttered\problem.json .\bin\Sample_03_Navigation\02_Navigation_cluttered\problem.json -r
	mkdir -p .\bin\Sample_03_Navigation\02_Navigation_cluttered\Results
	cp .\Samples\03_Navigation\print_result.js .\bin\Sample_03_Navigation\02_Navigation_cluttered\Results\print_result.js -r
	g++ -o ./bin/Sample_03_Navigation/02_Navigation_cluttered/02_Navigation_cluttered ./Samples/03_Navigation/02_Navigation_cluttered/Main.cpp ./Samples/src/geometry.cpp ./Samples/03_Navigation/src/Problem_Navigation.cpp ./bin/MT_RRT.lib -fopenmp -pthread -O3 -Wall

Sample_Navigation_GUI:
	mkdir -p .\bin\Sample_03_Navigation\00_GUI
	mkdir -p .\bin\Sample_03_Navigation\00_GUI\front_JS
	cp  .\Samples\03_Navigation\00_GUI\front_JS .\bin\Sample_03_Navigation\00_GUI\front_JS -r
	cp  .\Samples\03_Navigation\00_GUI\launcher.bat .\bin\Sample_03_Navigation\00_GUI\launcher.bat -r
	g++ -o ./bin/Sample_03_Navigation/00_GUI/back_Cpp ./Samples/03_Navigation/00_GUI/back_Cpp/Main.cpp ./Samples/src/geometry.cpp ./Samples/03_Navigation/src/Problem_Navigation.cpp ./Samples/src/Stream_Socket.cpp ./bin/MT_RRT.lib -fopenmp -pthread -lws2_32 -O3
	