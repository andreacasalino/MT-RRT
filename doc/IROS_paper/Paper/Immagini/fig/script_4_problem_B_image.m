clear; clc;
close all;

addpath('../../../../../Sample/Data/src');

Rob=import_Robot_Capsule('../../../../../Sample/Data/Robot/Scara_7_gdl.txt');
Scene=import_Obstacles('../../../../../Sample/Data/Scenarios/Scene_01/', 'Scene_01_Spheres_04.txt');

Qo=(pi/180)*[120, -30.0, -45.0, -40.0, 0.0, 0.0, -40.0];
Qf=(pi/180)*[15,-90,-50,-30,-90,0,-45];


figure;
hold on;
        plot_Obstacles(Scene, (1/255)*[128,0,128]);
plot_Rob_Traj(Qo, Rob, 'r', 1);
plot_Rob_Traj(Qf, Rob, 'r',1);
        axis equal;
set(gca,'XTickLabel',[])
set(gca,'YTickLabel',[])

rmpath('../../../../../Sample/Data/src');