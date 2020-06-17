clear; clc;
close all;

JSON_res=jsondecode(fileread('result.json'));

for k=1:length(JSON_res)
    if(length(JSON_res(k).sol) > 0)
        figure;
        hold on;
        axis equal;
        plot(JSON_res(k).sol(:,1) , JSON_res(k).sol(:,2), '-b', 'linewidth', 2);
        for p=1:5:size(JSON_res(k).sol,1)
            U = JSON_res(k).cost * cos(JSON_res(k).sol(p,3)) / (15);
            V = JSON_res(k).cost * sin(JSON_res(k).sol(p,3)) / (15);
            quiver(JSON_res(k).sol(p,1), JSON_res(k).sol(p,2), U, V, 'r');
        end
        disp(JSON_res(k).cost);
    end
end