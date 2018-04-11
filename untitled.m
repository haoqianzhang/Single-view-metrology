P1=[ -0.3317   -0.3033    0.0390  528.4539;
    -0.5512   -0.0033   -0.3133  714.1447;
    -0.0008    0.0000    0.0000    1.0000];
P=[1.4171   -1.3568    0.1920  883.0592;
    2.3550   -0.0149   -1.5404  780.9211;
    0.0034    0.0000    0.0002    1.0000];
P2=[1.1473    1.1695    0.1875  242.9276;
    1.9066    0.0129   -1.5047  771.7105;
    0.0027   -0.0000    0.0002    1.0000]
Hxy = [P(:,1:2),P(:,4)]
Hxz = [P(:,1),P(:,3:4)]
Hyz = P(:,2:4)
R=500;
%Hxyn = [P(:,1) P(:,2) P(:,3)*R+P(:,4)];
%Hxzn = [P(:,1) P(:,3) P(:,2)*R+P(:,4)];
Hyzn = [P(:,2) P(:,3) P(:,1)*R+P(:,4)];

img=imread('./paiting/paint.jpg');
invH=inv(Hyzn);
tform=zeros([1000,500,3]);
for i=1:500
    %for j=1:500
    for j=1:1000
        
        imgcoor=Hyzn*[j;i;1];
        imgcoor=round(imgcoor/imgcoor(3));
        %imgcoor
        tform(j,i,:)=img(imgcoor(2),imgcoor(1),:);
    end
end

tform=uint8(tform);
figure
imshow(tform);
[filename, pathname] = uiputfile({'*.jpg;','JPG files';'*.bmp','BMP files'}, 'Pick an Image');
if isequal(filename,0) || isequal(pathname,0)
    return;
else
    fpath=fullfile(pathname, filename);
end
imwrite(tform,fpath);


% xref=[500;500;1];
% img_xref=Hxy*xref;
% img_xref=img_xref/img_xref(3);
% invH
% invH = invH';
% tform = projective2d(invH);
% output = imwarp(img, tform);
% figure
% imshow(output);%, [I, ~] = imcrop(output);
% %delete(get(gca,'Children'));
% %imshow(I);
%
%
% Hxy1 =[    1.9905   -1.0636  807.6198;
%     3.3080   -0.0117  763.9434;
%     0.0047    0.0000    1.0000];
%
%
% img=imread('./paiting/paint.jpg');
% invH1=inv(Hxy1);
% xref=[500;500;1];
% img_xref=Hxy*xref;
% img_xref=img_xref/img_xref(3);
% invH1
% invH1 = invH1';
% tform = projective2d(invH1);
% output = imwarp(img, tform);
% figure
% imshow(output);%, [I, ~] = imcrop(output);
% %delete(get(gca,'Children'));
% %imshow(I);;
