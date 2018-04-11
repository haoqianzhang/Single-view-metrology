function vp=vanish_point(color)
disp('Set at least 3 lines for calculating vanish point')
lines = zeros(0,3);
M_sum = zeros(3,3);
points = zeros(0,4);
% get coordinates of line's two ends
while 1
    [x1, y1, b] = ginput(1);
    if b == 'q'
        break;
    end
    [x2, y2] = ginput(1);
    point = [x1, y1, x2, y2];
    points = [points;point];
    e1 = [x1,y1,1];
    e2 = [x2,y2,1];
    line = real(cross(e1,e2));
    lines = [lines;line];
    plot([x1 x2], [y1 y2], color, 'LineWidth', 2);
    M = line' * line;
    M_sum = M_sum+M;
end

[vp,~] = eigs(M_sum,1,'SM');
vp = vp/vp(3);
% g1=points(1,1);
% h1=points(1,2);
% g2=points(2,1);
% h2=points(2,2);
% plot([g1 vp(1)], [h1 vp(2)], color, 'LineWidth', 2);
% plot([g2 vp(1)], [h2 vp(2)], color, 'LineWidth', 2);
% 

    