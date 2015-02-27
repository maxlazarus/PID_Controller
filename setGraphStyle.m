function [] = setGraphStyle()
    whitebg('black');
    set(gca,'Xcolor',[0.5 0.5 0.5]);
    set(gca,'Ycolor',[0.5 0.5 0.5]);
    hold on;
    grid on;
    xlim([0 1]);
end

