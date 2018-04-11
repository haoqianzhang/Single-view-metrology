function I = texturemap(invH,img)
invH = invH';
tform = projective2d(invH);
output = imwarp(img, tform);
figure, [I, ~] = imcrop(output);
delete(get(gca,'Children'));
imshow(I);
end


% function texturemap(img, H)
% pos = ginput(4)
% %pos = reshape(pos, 2, 4)'; % 4 points on images
% minpos = min(pos);
% il = minpos(1);  %x
% jl = minpos(2);  %y
% maxpos = max(pos);
% ih = maxpos(1);  %x
% jh = maxpos(2);  %y
% points=[];
%texture = zeros(ih-il+1, jh-jl+1, 3);
% for i = 1:size(img,2)
%    for j = 1:size(img,1)
%        mapback = H\[i, j, 1]';
%        mapback = mapback/mapback(3);
%        round = floor(mapback);
%        x = round(1);
%        y = round(2);
%        points=[points;[x y]];
% %        a = mapback(1) - x;
% %        b = mapback(2) - y;
% %        texture(i-il+1, j-jl+1, :) =  (1-a)*(1-b)*img(y, x, :)...
% %                                    + a*(1-b)*img(y, x+1, :)...
% %                                    + a*b*img(y+1, x+1, :)...
% %                                    + b*(1-a)*img(y+1, x, :);
%    end
% end
% 
% for i = il:ih  %x
%    for j = jl:jh  %y
%        mapback = H\[i, j, 1]';
%        mapback = mapback/mapback(3);
%        round = floor(mapback);
%        x = round(1);
%        y = round(2);
%        points=[points;[x y]];
% %        a = mapback(1) - x;
% %        b = mapback(2) - y;
% %        texture(i-il+1, j-jl+1, :) =  (1-a)*(1-b)*img(y, x, :)...
% %                                    + a*(1-b)*img(y, x+1, :)...
% %                                    + a*b*img(y+1, x+1, :)...
% %                                    + b*(1-a)*img(y+1, x, :);
%    end
% end
% 
% minpos=min(points);
% maxpos=max(points);
% texture=zeros(maxpos(2)-minpos(2)+1,maxpos(1)-minpos(1)+1,3);
% % for i = 1:size(img,2)
% %     for j = 1:size(img,1)
%         row=size(img,2)*i;
%         point= points(row+j);
%         y = int32(point(1)-minpos(1)+1);
%         x = int32(point(2)-minpos(2)+1);
%         texture(y,x,:)=img(j,i,:);
%     end
% end 

% for j = 1:(jh-jl+1)  %y
%    for i = 1:(ih-il+1)  %x
%        num = (j-1)*(ih-il+1)+i;
%         point = points(num,:);
%         y = point(2)-minpos(2)+1;
%         x = point(1)-minpos(1)+1;
%         texture(y,x,:)=img(j+jl-1,i+il-1,:);
%     end
% end 
% 
% figure(2);
% textureout = uint8(texture);
% imwrite(textureout, 'texture.jpg');
% imshow(textureout);
%     
% 
% 
% end