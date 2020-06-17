function print__temp_tree()

T =  jsondecode(fileread('__temp_tree'));

figure;
hold on;
axis equal;
for k=1:length(T.Tree)
    x = [T.Tree(k).S(1), T.Tree(k).E(1)];
    y = [T.Tree(k).S(2), T.Tree(k).E(2)];
    plot(x,y,'-b');
    plot(x(2),y(2),'.g');
end
plot(T.Node_involved(1) , T.Node_involved(2), 'or', 'markersize', 10,'linewidth', 5);