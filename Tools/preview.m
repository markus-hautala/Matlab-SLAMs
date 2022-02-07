function preview(pcSet, time)
% Työkalu pistepilvisetin esikatseluun
preview_player = showPc('preview');

pause(5)
for item=1:length(pcSet)
    view(preview_player, pcSet{item});
    disp(item)
    pause(time)
end
end

