function [] = setGraphStyle(titleText)
    title(titleText, 'FontSize', 36);
    whitebg('black');
    set(gca,'Xcolor',[0.5 0.5 0.5]);
    set(gca,'Ycolor',[0.5 0.5 0.5]);
    grid on;
    xlim([0 1]);
    set(gcf,'units','normalized','outerposition',[0 0 1 1])
end

