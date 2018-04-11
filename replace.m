fid = fopen('2.wrl','w');
%fprintf(fid,'#VRML V2.0 utf8\n\nCollision {\n collide FALSE\n children [\n ]\n }');
fprintf(fid,'#VRML V2.0 utf8\n\nCollision {\n collide FALSE\n children [\n');
fclose(fid);
content=fileread('1.txt');
texture='xy.bmp';
result=strrep(content,'texture_source',texture)
a=[37.1630   20.6442   20.6442   37.1630;
    37.0847  397.9543  397.9543   37.0847;
    170.6014  170.6014         0         0];
b=a';
str=strjoin(string(c),',\n');
result=strrep(result,'geometry_coor', str)  % ??geo coord
fid = fopen('2.wrl','a');
fprintf(fid, result);
fclose(fid);
fid = fopen('2.wrl','a');
fprintf(fid, '\n ] \n}');
fclose(fid);