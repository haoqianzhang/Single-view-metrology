function varargout = proj3(varargin)
% PROJ3 MATLAB code for proj3.fig
%      PROJ3, by itself, creates a new PROJ3 or raises the existing
%      singleton*.
%
%      H = PROJ3 returns the handle to a new PROJ3 or the handle to
%      the existing singleton*.
%
%      PROJ3('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROJ3.M with the given input arguments.
%
%      PROJ3('Property','Value',...) creates a new PROJ3 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before proj3_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to proj3_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help proj3

% Last Modified by GUIDE v2.5 27-Mar-2018 18:31:07

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @proj3_OpeningFcn, ...
                   'gui_OutputFcn',  @proj3_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before proj3 is made visible.
function proj3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to proj3 (see VARARGIN)

% Choose default command line output for proj3
handles.output = hObject;
handles.vX=[];
handles.vY=[];
handles.vZ=[];
handles.O=[];
handles.Hxyn=[];
handles.axes1
handles.xref_len=[];
handles.yref_len=[];
handles.zref_len=[];
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes proj3 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = proj3_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in get_xref_co.
function get_xref_co_Callback(hObject, eventdata, handles)
% hObject    handle to get_xref_co (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isempty(handles.O)
    disp('Origin not initialized, select origin first!');
else
[x,y]=ginput(1);
plot(x,y,'*');
plot([handles.O(1) x], [handles.O(2) y], 'c', 'LineWidth', 2);
handles.xref=[x;y;1];
guidata(hObject, handles);
end


function get_xref_Callback(hObject, eventdata, handles)
% hObject    handle to get_xref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of get_xref as text
%        str2double(get(hObject,'String')) returns contents of get_xref as a double
xref_len = str2double(get(handles.get_xref, 'string'));
handles.xref_len=xref_len;
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function get_xref_CreateFcn(hObject, eventdata, handles)
% hObject    handle to get_xref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in get_yref_co.
function get_yref_co_Callback(hObject, eventdata, handles)
% hObject    handle to get_yref_co (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isempty(handles.O)
    disp('Origin not initialized, select origin first!');
else
[x,y]=ginput(1)
plot(x,y,'*');
plot([handles.O(1) x], [handles.O(2) y], 'c', 'LineWidth', 2);
handles.yref=[x;y;1];
guidata(hObject, handles);
end



function get_yref_Callback(hObject, eventdata, handles)
% hObject    handle to get_yref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of get_yref as text
%        str2double(get(hObject,'String')) returns contents of get_yref as a double
yref_len = str2double(get(handles.get_yref, 'string'));
handles.yref_len=yref_len;
guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function get_yref_CreateFcn(hObject, eventdata, handles)
% hObject    handle to get_yref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in get_zref_co.
function get_zref_co_Callback(hObject, eventdata, handles)
% hObject    handle to get_zref_co (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isempty(handles.O)
    disp('Origin not initialized, select origin first!');
else
    [x,y]=ginput(1)
    plot(x,y,'*');
    plot([handles.O(1) x], [handles.O(2) y], 'c', 'LineWidth', 2);
    handles.zref=[x;y;1];
    guidata(hObject, handles);
end


function get_zref_Callback(hObject, eventdata, handles)
% hObject    handle to get_zref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of get_zref as text
%        str2double(get(hObject,'String')) returns contents of get_zref as a double
zref_len = str2double(get(handles.get_zref, 'string'));
handles.zref_len=zref_len;
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function get_zref_CreateFcn(hObject, eventdata, handles)
% hObject    handle to get_zref (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white bac kground on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in select_origin.
function select_origin_Callback(hObject, eventdata, handles)
% hObject    handle to select_origin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('Select a point as origin');
[x, y]=ginput(1)
scatter(x,y,24,'y','fill')
handles.O=[x;y;1];
guidata(hObject, handles);



% --- Executes on button press in calculate_p_h.
function calculate_p_h_Callback(hObject, eventdata, handles)
% hObject    handle to calculate_p_h (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if (isempty(handles.vX)) || (isempty(handles.vY)) || (isempty(handles.vZ))
    msgbox('Initial information of parallel lines not complete!');
elseif (isempty(handles.xref)) || (isempty(handles.yref)) || (isempty(handles.zref))
    msgbox('Initial information of reference point not complete!');
elseif (isempty(handles.xref_len)) || (isempty(handles.yref_len)) || (isempty(handles.zref_len))
    msgbox('Initial lenght of reference point not complete!');
else
    disp('Start to calculate projection and homography matrixes');
    % get scale
    a = ((handles.vX - handles.xref)\ (handles.xref - handles.O))/handles.xref_len
    b = ((handles.vY - handles.yref)\ (handles.yref - handles.O))/handles.yref_len
    c = ((handles.vZ - handles.zref)\ (handles.zref - handles.O))/handles.zref_len
    P = [a*handles.vX b*handles.vY c*handles.vZ handles.O];
    handles.P=P;
    disp(P);
    P = [P(:,1) -P(:,2) -P(:,3) P(:,4)];
    % get homography matrix H
    
    Hxy = [P(:,1:2),P(:,4)]
    Hxz = [P(:,1),P(:,3:4)]
    Hyz = P(:,2:4)
    handles.Hxy = Hxy;
    handles.Hyz = Hyz;
    handles.Hxz = Hxz;
    guidata(hObject, handles);
end 


% --- Executes on button press in line_x.
function line_x_Callback(hObject, eventdata, handles)
% hObject    handle to line_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vp=vanish_point('r');
handles.vX=vp;
guidata(hObject, handles);


% --- Executes on button press in line_y.
function line_y_Callback(hObject, eventdata, handles)
% hObject    handle to line_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vp=vanish_point('g');
handles.vY=vp;
guidata(hObject, handles);


% --- Executes on button press in line_z.
function line_z_Callback(hObject, eventdata, handles)
% hObject    handle to line_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vp=vanish_point('b');
handles.vZ=vp;
guidata(hObject, handles);


% --- Executes on button press in save_vp.
function save_vp_Callback(hObject, eventdata, handles)
% hObject    handle to save_vp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vX=handles.vX;
vY=handles.vY;
vZ=handles.vZ;

[filename, pathname] = uiputfile({'*.mat'}, 'Select a mat file');  
if isequal(filename,0) || isequal(pathname,0)  
    return;
else  
    fpath=fullfile(pathname, filename);
end  
save(fpath, 'vX', 'vY', 'vZ');
disp('Vanish points have been saved');


% --- Executes on button press in load_vp.
function load_vp_Callback(hObject, eventdata, handles)
% hObject    handle to load_vp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile( ...  
    {'*.mat'},'Select File');  
  
if isequal(filename,0) || isequal(pathname,0)
    return;
else
    fpath=[pathname filename];
    load(fpath, 'vX', 'vY', 'vZ');
    handles.vX=vX;
    handles.vY=vY;
    handles.vZ=vZ;
    disp('Vanish points have been loaded');
    guidata(hObject, handles);
end


% --- Executes on button press in select_rp.
function select_rp_Callback(hObject, eventdata, handles)
% hObject    handle to select_rp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% vX=handles.vX;
% vY=handles.vY;
b0 = handles.O; % set origin point to b0
t0 = handles.zref; % set reference point in z axis to point t0
H = handles.zref_len; % set height in z axis to H
lvxy = cross(handles.vX,handles.vY);  % get vanish line between vX and vY points
disp('Choose one base point');
% --- below for mouse input coordinates ---
[xb,yb] = ginput(1);
b = [xb;yb;1];
plot(xb,yb,'o');
handles.b = b;
% ---above for mouse input coordinataes---

% ---below for manual input coordinataes---
% bpos = inputdlg('Enter base coordinate(xb, yb):','Input', [1 50]);
% bpos = str2num(bpos{:});
% b = [bpos(1);bpos(2);1];  % get base point b
% ---above for manual input coordinataes---

disp('Choose one reference point');
% --- below for mouse input coordinates ---
[xr,yr] = ginput(1);
r = [xr;yr;1];
plot(xr,yr,'o');
handles.r=r;
% ---above for mouse input coordinataes---

% ---below for manual input coordinataes---
% rpos = inputdlg('Enter reference coordinate(xr, yr):','Input', [1 50]);
% rpos = str2num(rpos{:});
% r = [rpos(1);rpos(2);1];  % get reference point r
% ---above for manual input coordinataes---

lb0b = cross(b0, b); % get line between b0 and b
v = cross(lb0b,lvxy); % get vanish point of line b0b and line vanish_xy
v = v / v(3);
lrb = cross(r, b); % get line between r and b
lvt0 = cross(v', t0); % get line between v and t0
%lor = cross(handles.origin, handles.ref);
t = cross(lvt0,lrb); % get point in t (intersection of line vt0 and line rb)
t = t / t(3);
% get height of point r
R = H*sqrt(sumsqr(r-b))*sqrt(sumsqr(handles.vZ'-t))/...
    sqrt(sumsqr(t'-b))/sqrt(sumsqr(handles.vZ-r))
handles.R=R;

disp('Input othter points that at the same height as reference point')

% --- below for mouse input coordinates ---
answer = input('Select othter points that at the same height as reference point? (Y/N) =>','s');
if char(answer) == 'Y'
    Hxyn = [handles.P(:,1) -handles.P(:,2) -handles.P(:,3)*R+handles.P(:,4)];
    handles.Hxyn = Hxyn;
    H = [handles.P(:,1) handles.P(:,2) handles.P(:,3)*R+handles.P(:,4)]
    newpos = H\r;
    worldCo = [newpos(1)/newpos(3);newpos(2)/newpos(3);R];
    oposworld = worldCo
    while 1
        disp('Select points at same height or press q to stop')
        [x,y,b] = ginput(1);
        if b=='q'
            break;
        end
        plot(x,y,'o');
        newp = [x;y;1];
        newpos = H\newp;
        worldCo = [newpos(1)/newpos(3);newpos(2)/newpos(3);R];
        oposworld = [oposworld worldCo]
    end
    handles.oposworld = oposworld;
    bposworld = oposworld;
    bposworld(3,:)=0;
    handles.bposworld = bposworld;
%     add=[handles.xref handles.yref handles.zref handles.O];
%     addworldCo=[];
%     for col=1:size(add,2)
%         newpos = handles.Hxy\add(:,col);
%         addCo = [newpos(1)/newpos(3);newpos(2)/newpos(3);0]
%         addworldCo=[addworldCo addCo];
%     end
%     handles.addworldCo=addworldCo;
    guidata(hObject, handles);
end

% ---above for mouse input coordinataes---

% ---below for manual input coordinataes---
% opos = inputdlg('Enter other coordinates at same height (x1, y1), (x2, y2),...:','Input', [1 70]);
% opos = str2num(opos{:});
% xopos = [];
% yopos = [];
% oposworld=[];
% Hxyn = [handles.P(:,1) handles.P(:,2) handles.P(:,3)*R+handles.P(:,4)];
% handles.Hxyn = Hxyn*[1 -1 -1];
% for i=1:(size(opos,2)/2)
%     x = opos(2*i-1);
%     y = opos(2*i);
%     xopos = [xopos x];
%     yopos = [yopos y];
%     newp = [x;y;1];
%     newpos = Hxyn\newp;
%     worldCo = [newpos(1)/newpos(3);newpos(2)/newpos(3);R];
%     oposworld = [oposworld worldCo];
% end
% plot(xopos, yopos,'w','LineWidth',0.1,'Marker', 'o', 'MarkerSize', 5, 'MarkerEdgeColor','r','MarkerFaceColor','r')
% ---above for manual input coordinataes---





% --- Executes on button press in create_vrml.
function create_vrml_Callback(hObject, eventdata, handles)
% hObject    handle to create_vrml (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% calculate world coordinates of base refxyz


% show all point
allpt = [handles.oposworld handles.bposworld];
pos = [allpt;1:size(allpt,2)]

% scatter plot
X = pos(1,:);
Y = pos(2,:);
Z = pos(3,:);
Order = pos(4,:);
textCell = arrayfun(@(x,y,z,order) sprintf('(%3.2f, %3.2f, %3.2f, p%d)',x,y,z,order),X,Y,Z,Order,'un',0);
figure;
scatter3(X,Y,Z);
for ii = 1:numel(X)  % add coordinates information to each point
    text(X(ii)+.02, Y(ii)+.02, Z(ii)+.02, textCell{ii},'FontSize',8) 
end

% choose coordinates for interested texture map
% valstring = inputdlg('Which coordinates to use for current texture map?(1,2,3,...,N)','Input', [1 70]);
% valparts = str2num(valstring{:});
% coord = str2double(valparts);
% num_coord = size(coord, 2);
% valstring = inputdlg('Input the texture file name with extension (e.g. xxx.bmp)','Input', [1 70]);
% cur_texture = imread(filename);
% cur_height = size(cur_texture, 1);
% cur_width = size(cur_texture, 2);
% figure, imshow(cur_texture);
% disp('Select texture coordinated in order');


% create new file to write vrml model
vrmlname = input('Input a name with extension for model? =>','s');
%vrmlname = inputdlg('Input a name with extension for model? =>','Input', [1 70]);
vrmlname = char(vrmlname);
ext='.wrl';
vrmlname=[vrmlname ext];
%if ~exist(vrmlname, 'file')
fid = fopen(vrmlname,'w');
%fprintf(fid,'#VRML V2.0 utf8\n\nCollision {\n collide FALSE\n children [\n ]\n }');
fprintf(fid,'#VRML V2.0 utf8\n\nCollision {\n collide FALSE\n children [\n');
fclose(fid);

num_tex = input('How many texutre map would be used? =>','s');
%num_tex = inputdlg('How many texutre map would be used? =>','Input', [1 70]);
num_tex = str2double(num_tex);
for num = 1:num_tex
    drawvrml(pos, vrmlname);
end
fid = fopen(vrmlname,'a');
fprintf(fid, '\n ] \n}');
fclose(fid);
% filecontent=fileread(vrmlname);
% create wrl file from ext file
% [~,filename,~]=fileparts(vrmlname);
% ext='.wrl';
% finalname=[filename ext];
% movefile(vrmlname, finalname);




% --- Executes on button press in create_xy.
function create_xy_Callback(hObject, eventdata, handles)
% hObject    handle to create_xy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% if isempty(handles.Hxyn)
%     disp('Base point and reference point not initialized, select base and reference points first!');
% else
    Hxyn = handles.Hxy;
    invH = inv (Hxyn);
    Txy = texturemap(invH, handles.img_src);
    handles.Txy = Txy;
    [filename, pathname] = uiputfile({'*.jpg;','JPG files';'*.bmp','BMP files'}, 'Pick an Image');
    if isequal(filename,0) || isequal(pathname,0)
        return;
    else
        fpath=fullfile(pathname, filename);
    end
    imwrite(Txy,fpath);
    %imwrite(Txy,'xy.jpg','jpg');
    guidata(hObject, handles);
%end


% --- Executes on button press in create_yz.
function create_yz_Callback(hObject, eventdata, handles)
% hObject    handle to create_yz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% if isempty(handles.Hxyn)
%     disp('Base point and reference point not initialized, select base and reference points first!');
% else
    Hyz = handles.Hyz;
    invH = inv(Hyz);
    Tyz = texturemap(invH, handles.img_src);
    handles.Tyz = Tyz;
    [filename, pathname] = uiputfile({'*.jpg;','JPG files';'*.bmp','BMP files'}, 'Pick an Image');
    if isequal(filename,0) || isequal(pathname,0)
        return;
    else
        fpath=fullfile(pathname, filename);
    end
    imwrite(Tyz,fpath);
    %imwrite(Tyz,'yz.jpg','jpg');
    guidata(hObject, handles);
%end


% --- Executes on button press in create_xz.
function create_xz_Callback(hObject, eventdata, handles)
% hObject    handle to create_xz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% if isempty(handles.Hxyn)
%     disp('Base point and reference point not initialized, select base and reference points first!');
% else
    Hxz = handles.Hxz;
    invH = inv(Hxz);
    Txz = texturemap(invH, handles.img_src);
    handles.Txz = Txz;
    [filename, pathname] = uiputfile({'*.jpg;','JPG files';'*.bmp','BMP files'}, 'Pick an Image');
    if isequal(filename,0) || isequal(pathname,0)
        return;
    else
        fpath=fullfile(pathname, filename);
    end
    imwrite(Txz,fpath);
    %imwrite(Txz,'xz.jpg','jpg');
    guidata(hObject, handles);
%end


% --------------------------------------------------------------------
function open_file_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to open_file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile( ...  
    {'*.bmp;*.jpg;*.png;*.jpeg','Image Files (*.bmp, *.jpg, *.png,*.jpeg)'; ...  
    '*.*',                'All Files (*.*)'}, ...  
    'Pick an image');  
  
if isequal(filename,0) || isequal(pathname,0)
    return;
else
    fpath=[pathname filename];
    img_src=imread(fpath);
    handles.img_src=img_src;
    %[handles.height, handles.width, handles.dim] = size(handles.img_src);
    %set(handles.axes1,'position',[4.4, 2.8, handles.width, handles.height]);
    axes(handles.axes1);
    hold all;
    %imshow(handles.img_src);
    delete(get(gca,'Children'));
    imshow(handles.img_src);
    
    %plots = [];
    %axis on;
    guidata(hObject, handles);
    
end
