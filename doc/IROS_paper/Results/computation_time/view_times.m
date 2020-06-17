function  view_times(Data, Col)

Example=length(Data);

offset=0.2/(Example);

for ke=1:Example
    Mean_serial=mean(  Data{ke}(:,1) );
    Dat=Data{ke}(:,2:end);
    for kt=1:size(Dat,2)
        Dat(:,kt)=(1./(  (1+kt) *   Dat(:,kt)  ))*Mean_serial;
    end
        
    Mean_val=mean(Dat);
    x_pos=[1:size(Data{ke},2)]+ offset*(ke-1);
    
    plot(x_pos, [1,Mean_val],'Color', Col{ke},'linewidth',2);
    
    for kt=1:size(Dat,2)
        plot_single_thread(x_pos(kt+1), Dat(:,kt) , Col{ke}, 0.2);
    end
end

end

function plot_single_thread(x_pos, vals, Col, L)

min_val=min(vals);
max_val=max(vals);

plot([x_pos, x_pos], [min_val, max_val],'Color', Col,'linewidth',2);
plot([x_pos-0.5*L, x_pos+0.5*L], [min_val,min_val],'Color', Col,'linewidth',2);
plot([x_pos-0.5*L, x_pos+0.5*L], [max_val,max_val],'Color', Col,'linewidth',2);

end