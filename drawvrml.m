function drawvrml(pos, vrmlname)
val = inputdlg('Which coordinates to use for current texture map?(1,2,3,...,N)','Input', [1 70]);
coord = str2num(val{:})
num_coord = size(coord, 2);
disp('Input the texture file');
[filename, pathname] = uigetfile( ...
    {'*.bmp;*.jpg;*.png;*.jpeg','Image Files (*.bmp, *.jpg, *.png,*.jpeg)'; ...
    '*.*',                'All Files (*.*)'}, ...
    'Pick an image');
if isequal(filename,0) || isequal(pathname,0)
    return;
else
    fpath=[pathname filename];
    cur_texture=imread(fpath);
    %filename = inputdlg('Input the texture file name with extension (e.g. xxx.bmp)','Input', [1 70]);
    %filename = char(filename);
    %cur_texture = imread(filename);
    cur_height = size(cur_texture, 1);
    cur_width = size(cur_texture, 2);
    figure, imshow(cur_texture);
    disp('Select texture coordinated in order');
    texCoor = zeros(2,0);
    
    
    % map texture coordinates with real world coordinates
    for i = 1:num_coord
        [x,y] = ginput(1);
        %plot(x,y,'*');
        texCoor(:, end+1) = [round(x/cur_width); round((cur_height-y)/cur_height)]
    end
    
    % fcontent = fileread(vrmlname);
    % fid = fopen(vrmlname,'w');
    % if isempty(fcontent)
    %     fprintf(fid,'#VRML V2.0 utf8\n\nCollision {\n collide FALSE\n children [\n');
    %     fclose(fid);
    % end
    
    % append a new shape and add last two lines (] &})
    %fid = fopen(vrmlname,'a');
    content=fileread('format.txt'); % load style example
    % replace file name
    result=strrep(content,'texture_source',filename);
    
    % replace geometry coord coordinates
    curpos = zeros(4,0);
    for i = 1:num_coord
        curpos = [curpos pos(:,coord(i))];
    end
    curpos=curpos';
    curpos=curpos(:,1:3);
    curpos=num2str(curpos);
    pos_str=strjoin(string(curpos),',\n');
    result=strrep(result,'geometry_coord', pos_str);
    
    % repalce coordindex
    i = 1:num_coord;
    i = i-1;
    i = [i -1];
    stri=strjoin(string(i),',');
    result=strrep(result,'coord_index', stri);
    
    % replace texture coordinates
    texCoor=texCoor';
    texCoor=num2str(texCoor);
    texCoor_str=strjoin(string(texCoor),',\n');
    result=strrep(result,'tex_coord', texCoor_str);
    
    result=strrep(result,'tex_index', stri);
    
    fid = fopen(vrmlname,'a');
    fprintf(fid, result);
    fclose(fid);
end

% fid = fopen(vrmlname,'a');
% fprintf(fid, '\n Shape {\n  appearance Appearance {\n   texture ImageTexture {\n   url "');
% fprintf(fid, filename);
% fprintf(fid, '"\n  }  \n  }\n   geometry IndexedFaceSet {\n   coord Coordinate {\n   point [\n');
% for i = 1:num_coord
%     cur = coord(i);
%     fprintf(fid, '     %9.5f %9.5f %9.5f, \n', pos(1,cur),pos(2,cur),pos(3,cur));
% end
% fprintf(fid, '\n   ]\n   }\n   coordIndex [\n    ');
% for i = 1:num_coord
%     fprintf(fid, '%i,', i-1);
% end
% fprintf(fid, '%i,', -1);
% fprintf(fid, '\n   ]\n   texCoord TextureCoordinate {\n    point [\n');
% for i = 1:num_coord
%     fprintf(fid, '     %3.2f %3.2f,\n', texCoor(1,i), texCoor(2,i));
% end
% fprintf(fid, '\n    ]\n   }\n   texCoordIndex [\n    ');
% for i = 1:num_coord
%     fprintf(fid, '%i,', i-1);
% end
% fprintf(fid, '%i,', -1);
% fprintf(fid, '\n   ]\n   solid FALSE\n  }\n}');
% fclose(fid);
