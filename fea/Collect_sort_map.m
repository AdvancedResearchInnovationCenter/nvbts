 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
clc; 

filelist = fullfile('**\Data_Arr.mat');
filelist = dir(filelist);
EE= [];
HH= [];
TT= [];
Ang_X = [];
kk = 0;
Ang_Z = []; 
Data = struct;
for i_folder = 1 : size(filelist,1)
    subfolder = filelist(i_folder).folder;
%     m = regexp(string(subfolder),'E=(\d+)kpa\\H=(\d+.\d+)mm\\Ang_X=(\d+.\d+)\\Ang_Z=(\d+.\d+)\\T=(\d+.\d+)mm','tokens');
    m = regexp(string(subfolder),'Dist=(\d+)markers\\H=(\d+.\d+)mm\\Ang_X=(\d+.\d+)\\Ang_Z=(\d+.\d+)\\T=(\d+.\d+)mm','tokens');
    EE = [EE,str2num(m{1}{1})]; % distribution 
    HH = [HH,str2num(m{1}{2})];
    Ang_X = [Ang_X,str2num(m{1}{3})];
    Ang_Z = [Ang_Z,str2num(m{1}{4})];
    TT = [TT,str2num(m{1}{5})];
    if rem(str2num(m{1}{3}),2) == 1
        subfolder;
    end
end

EE = unique(EE);
HH = unique(HH);
TT = unique(TT);
Ang_X = unique(Ang_X);
Ang_Z = unique(Ang_Z);
% sym_ang = max(Ang_Z);
% Ang_Z = [   Ang_Z  -flip(Ang_Z(2:end-1))];
% Ang_Z = [ -flip(Ang_Z(2:end-1))+360  Ang_Z  ];

Test_Arr = load([filelist(1).folder,'\Data_Arr.mat']).marker.x;

Matrix_size = [length(EE),length(HH),length(TT),length(Ang_X),length(Ang_Z),size(Test_Arr,2),size(Test_Arr,1)];

XX=zeros(Matrix_size);
YY=zeros(Matrix_size);
ZZ=zeros(Matrix_size);
ZZ_RP=zeros(length(EE),length(HH),length(TT),length(Ang_X),length(Ang_Z),size(Test_Arr,2));
XX_sorted=zeros(Matrix_size);
YY_sorted=zeros(Matrix_size);
ZZ_sorted=zeros(Matrix_size);
UU=zeros(Matrix_size);
VV=zeros(Matrix_size);
RR=zeros(Matrix_size);
Theta=zeros(Matrix_size);
RR_Sec=zeros(Matrix_size);
Theta_Sec=zeros(Matrix_size);
UV_data = zeros(length(EE)*length(HH)*length(TT)*length(Ang_X)*length(Ang_Z)*21, 6+169*2);


DX=zeros(length(EE),length(HH),length(TT),length(Ang_X),length(Ang_Z),size(Test_Arr,1));
DY=zeros(length(EE),length(HH),length(TT),length(Ang_X),length(Ang_Z),size(Test_Arr,1));

% Constants; % recall C2S and the camera parameters
% Proj_mat = load('projection_matrix.mat').P;
w = waitbar(0, 'Starting');
for i_folder = 1 : size(filelist,1)   % load the data from Arrays in ABAQUS directory
    subfolder = filelist(i_folder).folder;
    m = regexp(string(subfolder),'Dist=(\d+)markers\\H=(\d+.\d+)mm\\Ang_X=(\d+.\d+)\\Ang_Z=(\d+.\d+)\\T=(\d+.\d+)mm','tokens');
    % indecis
    ee = find(EE == str2double(m{1}{1}));
    hh = find(HH == str2double(m{1}{2}));
    ax = find(double(Ang_X) == str2double(m{1}{3}));
    az = find(Ang_Z ==  str2double(m{1}{4}));
    tt = find(TT ==  str2double(m{1}{5}));
    path = filelist(i_folder).folder;
    Arr = load([path,'\Data_Arr.mat']);
    data = Arr.marker;


    ss = EE(ee);
    t_f = size(squeeze(data.x(:,:,2))',1);
    XX(ee,hh,tt,ax,az,1:t_f,1:ss) =   squeeze(data.x(:,:,2))' ;
    YY(ee,hh,tt,ax,az,1:t_f,1:ss) =   squeeze(data.y(:,:,2))' ;
    ZZ(ee,hh,tt,ax,az,1:t_f,1:ss) =   squeeze(data.z(:,:,2))' ;

    
    waitbar(i_folder/size(filelist,1), w, sprintf('Collecting [X,Y,Z] from files: %d %%', floor(i_folder/size(filelist,1)*100)));


end
close(w)
%%  remove the contact point 

% XX = XX(:,:,:,:,:,:,1:end-1);
% YY = YY(:,:,:,:,:,:,1:end-1);
% ZZ = ZZ(:,:,:,:,:,:,1:end-1);


%% repeat the sector of data by rotation about Z-axis of times (N_Sec)
repeat = 0;     % 1: yes, 0: no
if repeat == 1
Ang_Sec = Ang_Z(2)+ Ang_Z(end); 
N_Sec = 360 / Ang_Sec;
Ang_Z_All = [];

for i = 1 : N_Sec
    Ang_Z_All = cat(2,Ang_Z_All, ((i-1)*Ang_Sec)+Ang_Z);
end
Ang_Z = Ang_Z_All;
sx = size(XX); sx(5) = sx(5) * N_Sec;  % increase the matrix dim to full the whole rotation
sx_ref =  size(xx_ref); sx_ref(5) = sx_ref(5)*N_Sec;   % increase the matrix dim to full the whole rotation
sx_cnt =  size(xx_ref); sx_cnt(5) = sx_cnt(5)*N_Sec;   % increase the matrix dim to full the whole rotation

XX_All = zeros(sx); YY_All = XX_All; ZZ_All = XX_All;
xx_ref_All = zeros(sx_ref); yy_ref_All = xx_ref_All; zz_ref_All = xx_ref_All;
xx_cnt_All = zeros(sx_cnt); yy_cnt_All = xx_cnt_All; zz_cnt_All = xx_cnt_All;

NperSec = size(XX,5) ;  % number of angular steps arounf Z-axis per sector 
w = waitbar(0, 'Starting');
N_waitbar = N_Sec*size(XX,1)*size(XX,2)*size(XX,3)*size(XX,4)*size(XX,5);
i_waitbar = 1;
for n = 1 : N_Sec
for e = 1 : size(XX,1)
    for h = 1 : size(XX,2)
        for t = 1 : size(XX,3)
            for ax = 1 : size(XX,4)
                for az = 1 : size(XX,5)
                    for f = 1 : size(XX,6)
                        
                        rot = [ cosd(Ang_Sec*(n-1)) -sind(Ang_Sec*(n-1))    0 ;
                                sind(Ang_Sec*(n-1)) cosd(Ang_Sec*(n-1))     0 ;
                                0                  0            1 ];           % Rotation matrix abot Z-axis

                        xyz_rotated = rot * [ squeeze(XX(e,h,t,ax,az,f,:))'  ;
                                              squeeze(YY(e,h,t,ax,az,f,:))'  ;
                                              squeeze(ZZ(e,h,t,ax,az,f,:))' ];

                        XX_All(e,h,t,ax,az+((n-1)*NperSec),f,:) = xyz_rotated(1,:);
                        YY_All(e,h,t,ax,az+((n-1)*NperSec),f,:) = xyz_rotated(2,:);
                        ZZ_All(e,h,t,ax,az+((n-1)*NperSec),f,:) = xyz_rotated(3,:);
                        
                        % ref point on the wall palne 
                        xyz_ref_rotated = rot * [squeeze(xx_ref(e,h,t,ax,az,f,:))'  ;
                                                 squeeze(yy_ref(e,h,t,ax,az,f,:))'  ;
                                                 squeeze(zz_ref(e,h,t,ax,az,f,:))'  ];

                        xx_ref_All(e,h,t,ax,az+((n-1)*NperSec),f,:) = xyz_ref_rotated(1,:);
                        yy_ref_All(e,h,t,ax,az+((n-1)*NperSec),f,:) = xyz_ref_rotated(2,:);
                        zz_ref_All(e,h,t,ax,az+((n-1)*NperSec),f,:) = xyz_ref_rotated(3,:);
                        
                        % contact point between the palne and the wall 
                        xyz_cnt_rotated = rot * [squeeze(xx_cnt(e,h,t,ax,az,f,:))'  ;
                                                 squeeze(yy_cnt(e,h,t,ax,az,f,:))'  ;
                                                 squeeze(zz_cnt(e,h,t,ax,az,f,:))'  ];

                        xx_cnt_All(e,h,t,ax,az+((n-1)*NperSec),f,:) = xyz_cnt_rotated(1,:);
                        yy_cnt_All(e,h,t,ax,az+((n-1)*NperSec),f,:) = xyz_cnt_rotated(2,:);
                        zz_cnt_All(e,h,t,ax,az+((n-1)*NperSec),f,:) = xyz_cnt_rotated(3,:);

                    end
                    waitbar(i_waitbar/N_waitbar, w, sprintf('Repeating sectors by rotation: %d %%', floor(i_waitbar/N_waitbar*100)));
                    i_waitbar = i_waitbar +1 ;
                end
            end
        end
    end
end
end
close(w)


% % Visualize whole dataset
% figure
% for i = 1:72
%     scatter3(squeeze(XX_All(1,1,1,1,i,5,:)),squeeze(YY_All(1,1,1,5,i,end,:)),squeeze(ZZ_All(1,1,1,5,i,end,:)))
%     xlabel("x")
%     ylabel("y") 
%     xlim([-0.025 0.025])
%     ylim([-0.025 0.025])
%     pause(0.005)
% end
%% 
elseif repeat == 0
XX_All=XX;
YY_All=YY;
ZZ_All=ZZ;
Ang_Z_All =Ang_Z;

end
%% Sorting
XX_All_sorted = zeros(size(XX_All));
YY_All_sorted = zeros(size(YY_All));
ZZ_All_sorted = zeros(size(ZZ_All));
w = waitbar(0, 'Starting');
N_waitbar = size(XX_All,1)*size(XX_All,2)*size(XX_All,3)*size(XX_All,4)*size(XX_All,5) ;
i_waitbar = 1;
iii = 1;
Ln = size(XX_All,1)*size(XX_All,2)*size(XX_All,3)*size(XX_All,4)*size(XX_All,5)*size(XX_All,6);
XYZ = zeros(Ln,6+3*169);
for e = 1 : size(XX_All,1)
    for h = 1 : size(XX_All,2)
        for t = 1 : size(XX_All,3)
            for ax = 1 : size(XX_All,4)
                for az = 1 : size(XX_All,5)
                        X_1st_frame = squeeze(XX_All(e,h,t,ax,az,1,1:EE(e)))';   % first frame coordinates
                        Y_1st_frame =  squeeze(YY_All(e,h,t,ax,az,1,1:EE(e)))';
                        Z_1st_frame =  squeeze(ZZ_All(e,h,t,ax,az,1,1:EE(e)))';
                        
                    
                        R = sqrt(X_1st_frame.^2+Y_1st_frame.^2);
                        Theta = atan2(Y_1st_frame,X_1st_frame);
                        Theta(Theta<-0.01) = Theta(Theta<-0.005) +2*pi ;

                        %%Run k-means clustering to cluster according to the radius
                        N_patterns = [8];

                        [idx, C] = kmeans(R', N_patterns(e));   % 8 IS THE NUMBER Of radii including the center point
                    
                        hhh = histcounts(idx);
                        [ii,mm] = sort(hhh);
                        counter = 0 ;
                        idx = idx * 10 ;
                    
                        for i = 1:size(ii,2) 
                            [~,idd] = ismember(idx,mm(i)*10);
                            idx(idd>0) = i;
                        end
                    
                        [ ~ , index  ] = sortrows([idx,Theta'],[1,2],"ascend");
                        XX_All_sorted(e,h,t,ax,az,:,1:EE(e))= squeeze(XX_All(e,h,t,ax,az,:,index));
                        YY_All_sorted(e,h,t,ax,az,:,1:EE(e))= squeeze(YY_All(e,h,t,ax,az,:,index));
                        ZZ_All_sorted(e,h,t,ax,az,:,1:EE(e))= squeeze(ZZ_All(e,h,t,ax,az,:,index));
                        
                        xx = squeeze(XX_All(e,h,t,ax,az,:,index));
                        yy = squeeze(YY_All(e,h,t,ax,az,:,index));
                        zz = squeeze(ZZ_All(e,h,t,ax,az,:,index));
                        for f = 1 : size(XX_All,6)
                            XYZ(iii,:) = [EE(e), HH(h), TT(t), Ang_X(ax), Ang_Z(az), f, xx(f,:), yy(f,:), zz(f,:)];
                            iii = iii +1 
                        end
                        waitbar(i_waitbar/N_waitbar, w, sprintf('Sorting: %d %%', floor(i_waitbar/N_waitbar*100)));
                        i_waitbar = i_waitbar +1 ;
                end
            end
        end
    end
end


close(w)
%%
b = false;
if b == true
figure
for i = 1 : 50 
    scatter(squeeze(XX_All_sorted(ee,hh,tt,ax,az,21,i)), squeeze(YY_All_sorted(ee,hh,tt,ax,az,21,i)))%,squeeze(ZZ_All_sorted(ee,hh,tt,ax,az,1,i)))
    hold on
    pause(0.01)
end
end
%%
clear('XX_All','YY_All','ZZ_All');
XX_All = XX_All_sorted ; YY_All = YY_All_sorted ; ZZ_All = ZZ_All_sorted ;



%% generate and save the Parameters (P)
P.Ang_Z = Ang_Z;

CK = replace(sprintf('Case-%s',datetime('now')),":","-");

if ~exist('data')
    mkdir('data')
end 
if ~exist('data/last')
    mkdir('data/last')
end


try
    movefile('data/last/*',['data/backup/',CK]) 
catch
end
    
Rx = data.Rx(1,:,2);
Ry = data.Ry(1,:,2);
Rz = data.Rz(1,:,2);

save(fullfile('data/last','Z_All.mat'),'ZZ_All');
save(fullfile('data/last','Y_All.mat'),'YY_All');
save(fullfile('data/last','X_All.mat'),'XX_All');
save(fullfile('data/last','XYZ.mat'),'XYZ');
save(fullfile('data/last','Rx.mat'),'Rx');
save(fullfile('data/last','Ry.mat'),'Ry');
save(fullfile('data/last','Rz.mat'),'Rz');


P = struct;  %% parameters
P.E = EE;
P.H = HH;
P.T = TT;
P.Ang_X = Ang_X;
P.Ang_Z = Ang_Z_All;
save(fullfile('data/last','P.mat'),'P');

