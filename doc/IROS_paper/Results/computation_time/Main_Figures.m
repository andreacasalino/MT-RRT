clear; clc;
close all;

folders={'Abstract_Control','Path_plan','Traj_plan','Path_plan_7gdl'};

strategies={'MultiT_parallel_search','MultiT_shared_list','MultiT_copied_trees','MultiT_master_slave'};

for kf=1:length(folders)
    temp=Read_Time([folders{kf},'/Serial']);
    for kk=1:length(temp)
        Time_serial{kf,kk}=temp{kk};
    end
end
Col{1}=(1/255)*[255,0,0];
Col{2}=(1/255)*[128,0,128];
Col{3}=(1/255)*[87,227,224];
Col{4}=(1/255)*[255,128,0];
for ks=1:length(strategies)
    for kf=1:length(folders)
        Data=Read_Time([folders{kf},'/',strategies{ks}]);
    
        Time{ks,1}{kf}=[Time_serial{kf,1},Data{1}];
        if(length(Data)==3)
            Time{ks,2}{kf}=[Time_serial{kf,2},Data{2}];
            Time{ks,3}{kf}=[Time_serial{kf,3},Data{3}];
        else
            Time{ks,3}{kf}=[Time_serial{kf,3},Data{2}];
        end
    end
    
    h=figure;
    hold on;
    ylabel('$\xi$','Fontsize',18,'Interpreter', 'latex');
    title('RRT','Fontsize',18);
    view_times(Time{ks,1}, Col);
    grid on;
    set(gca,'xtick',[1,2,3,4]);
    set(gca,'FontSize',18);
    xlabel('Number of threads','FontSize',18);
axis([0.7 4.3 -inf inf]);
    %savefig(['../../Paper/Immagini/fig/time_',num2str(ks),'_',num2str(1)]);
    %saveas(h,['../../Paper/Immagini/pdf/time_',num2str(ks),'_',num2str(1)],'eps'); 

    if(length(Data)==3)
            h=figure;
            hold on;
            title('RRT bid.','Fontsize',18);
            view_times(Time{ks,2}, Col);
            grid on;
    set(gca,'xtick',[1,2,3,4]);
    set(gca,'FontSize',18);
    xlabel('Number of threads','FontSize',18);
axis([0.7 4.3 -inf inf]);
           %savefig(['../../Paper/Immagini/fig/time_',num2str(ks),'_',num2str(2)]);
           % saveas(h,['../../Paper/Immagini/pdf/time_',num2str(ks),'_',num2str(2)],'eps');
            
            h=figure;
            hold on;
            title('RRT*','Fontsize',18);
            view_times(Time{ks,3}, Col);
            grid on;
    set(gca,'xtick',[1,2,3,4]);
    set(gca,'FontSize',18);
    xlabel('Number of threads','FontSize',18);
axis([0.7 4.3 -inf inf]);
            %savefig(['../../Paper/Immagini/fig/time_',num2str(ks),'_',num2str(3)]);
            %saveas(h,['../../Paper/Immagini/pdf/time_',num2str(ks),'_',num2str(3)],'eps');
    else
            h=figure;
            hold on;
            title('RRT*','Fontsize',18);
            view_times(Time{ks,3}, Col);
            grid on;
    set(gca,'xtick',[1,2,3,4]);
    set(gca,'FontSize',18);
    xlabel('Number of threads','FontSize',18);
axis([0.7 4.3 -inf inf]);
            %savefig(['../../Paper/Immagini/fig/time_',num2str(ks),'_',num2str(3)]);
           % saveas(h,['../../Paper/Immagini/pdf/time_',num2str(ks),'_',num2str(3)],'eps');
    end
    
end
clear Data;

figure;
hold on;
plot(0,0,'Color',Col{1});
plot(0,0,'Color',Col{2});
plot(0,0,'Color',Col{3});
plot(0,0,'Color',Col{4});
[~,hObj]=legend({'Problem 4','Problem 1','Problem 3','Problem 2'},'FontSize',18);
hL=findobj(hObj,'type','line');  % get the lines, not text
set(hL,'linewidth',4)   