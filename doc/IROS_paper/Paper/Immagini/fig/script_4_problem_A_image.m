clear; clc;
close all;

addpath('../../../../../Sample/Data/src');

Rob=import_Robot_Capsule('../../../../../Sample/Data/Robot/Scara_3_gdl.txt');
Scene=import_Obstacles('../../../../../Sample/Data/Scenarios/Scene_01/', 'Scene_01_Spheres_04.txt');

Path_star=load('../../../../../Sample/Sample_Path_planning/Analysis/Serial_version/path_star.txt');
Path_star=Interpolate_Traj(Path_star,10);

figure;
hold on;
        plot_Obstacles(Scene, (1/255)*[128,0,128]);
plot_Rob_Traj(Path_star(1,1:3), Rob, 'r', 1);
plot_Rob_Traj(Path_star(end,1:3), Rob, 'r',1);
        axis equal;
set(gca,'XTickLabel',[])
set(gca,'YTickLabel',[])

figure;
hold on;
        plot_Obstacles(Scene, (1/255)*[128,0,128]);
plot_Rob_Traj(Path_star(1:end,1:3), Rob, 'b',1);
plot_Rob_Traj(Path_star(1,1:3), Rob, 'r',1);
plot_Rob_Traj(Path_star(end,1:3), Rob, 'r',1);
        axis equal;
set(gca,'XTickLabel',[])
set(gca,'YTickLabel',[])


addpath('../../../../../Sample/Sample_Path_planning/Analysis/Optimality_check/');
for k=1:4
    path{k}.Q=load(['../../../../../Sample/Sample_Path_planning/Analysis/Optimality_check/path_normal_',num2str(k-1)]);
    path{k}.C=cost_eval(path{k}.Q);
    path{k}.Q=Interpolate_Traj(path{k}.Q,10);
    
    path_ants{k}.Q=load(['../../../../../Sample/Sample_Path_planning/Analysis/Optimality_check/path_ants_star_',num2str(k-1)]);
    path_ants{k}.C=cost_eval(path_ants{k}.Q);
    path_ants{k}.Q=Interpolate_Traj(path_ants{k}.Q,10);
end
rmpath('../../../../../Sample/Sample_Path_planning/Analysis/Optimality_check/');


figure;
hold on;
        plot_Obstacles(Scene, (1/255)*[128,0,128]);
plot_Rob_Traj(Path_star(1,1:3), Rob, 'r', 1);
plot_Rob_Traj(Path_star(end,1:3), Rob, 'r',1);
for kp=1:length(path)
    plot_Rob_EE_Traj(path{kp}.Q , Rob, 'b');
end
        axis equal;
set(gca,'XTickLabel',[])
set(gca,'YTickLabel',[])

figure;
hold on;
        plot_Obstacles(Scene, (1/255)*[128,0,128]);
plot_Rob_Traj(Path_star(1,1:3), Rob, 'r', 1);
plot_Rob_Traj(Path_star(end,1:3), Rob, 'r',1);
for kp=1:length(path_ants)
    plot_Rob_EE_Traj(path_ants{kp}.Q , Rob, 'b');
end
        axis equal;
set(gca,'XTickLabel',[])
set(gca,'YTickLabel',[])

rmpath('../../../../../Sample/Data/src');