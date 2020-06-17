clear; clc;
close all;

addpath('../../../../../Sample/Data/src');

Rob=import_Robot_Capsule('../../../../../Sample/Data/Robot/Scara_3_gdl.txt');
Scene=import_Obstacles('../../../../../Sample/Data/Scenarios/Scene_01/', 'Scene_01_Spheres_04.txt');

Path_path=load('../../../../../Sample/Sample_Path_planning/Analysis/Serial_version/path_star.txt');
Path_path=Interpolate_Traj(Path_path,10);
Path_kyno=load('../../../../../Sample/Sample_Kyno_version/Analysis/path_computed');
Path_kyno=Interpolate_Traj(Path_kyno,10);

figure;
hold on;
        plot_Obstacles(Scene, (1/255)*[128,0,128]);
 plot_Rob_EE_Traj(Path_path , Rob, 'b');
 plot_Rob_EE_Traj(Path_kyno , Rob, 'cyan');
 axis equal;
set(gca,'XTickLabel',[])
set(gca,'YTickLabel',[])


Tree=load('../../../../../Sample/Sample_Kyno_version/Analysis/Tree_computed');

Qf=(pi/180)*[15,-90,-50]';
figure;
hold on;
axis equal;
%plot3(Tree(:,1),Tree(:,2),Tree(:,3),'.b');
for k=2:size(Tree,1)
    line=[Tree(k,1:3);
          Tree(k,7:9)];
    plot3(line(:,1),line(:,2),line(:,3),'b-');  
end
plot3(Qf(1),Qf(2),Qf(3),'sq','markersize',6,'linewidth',6,'Color','r');
plot3(Tree(1,1),Tree(1,2),Tree(1,3),'sq','markersize',6,'linewidth',6,'Color','r');
view([-47,63]);
set(gca,'XTickLabel',[])
set(gca,'YTickLabel',[])


rmpath('../../../../../Sample/Data/src');