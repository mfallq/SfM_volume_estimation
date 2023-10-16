%% Plot raw acc from mobile phone sensor
close all;clear all;clc
lineW=3;
FontS = 30;
fid=fopen('D:\Lagring\Plugg\Examensarbete\Data\images\pile2-2\rawacceleration.txt');
%fid=fopen('C:\Users\Marcus\Desktop\Examensarbete\Data\rawacceleration.txt');

s=textscan(fid,'%f %f %f','headerlines',0);
fclose(fid);

N=length(s{1});
time = 0:N-1;
time=time*0.01;

x=s{1};
y=s{2};
z=s{3};
rawacc=[x y z];

figure(5)
plot(time,rawacc,'linewidth',lineW)
set(gca,'fontsize',20)
lgd = legend({'X','Y','Z'});
lgd.FontSize = FontS;
title('Raw readings','FontSize',FontS)
xlabel('Time [s]','FontSize',FontS)
ylabel('Raw acceleration readings [m/s^2]','FontSize',FontS)

%{
plot(x(1:N));
title('Raw acc X-axis')
xlabel('Raw acc X-axis')
figure(6)
plot(y(1:N));
title('Raw acc Y-axis')
xlabel('Raw acc Y-axis')
figure(7)
plot(z(1:N));
title('Raw acc Z-axis')
xlabel('Raw acc Z-axis')
%}
%% Plot acc from IMU when rotated using gyro and gravitation information
%close all;
fid=fopen('D:\Lagring\Plugg\Examensarbete\Data\images\pile2-2\acceleration.txt');
%fid=fopen('C:\Users\Marcus\Desktop\Examensarbete\Data\acceleration.txt');

s=textscan(fid,'%f %f %f','headerlines',0);
fclose(fid);
x=s{1};
N=length(s{1});
y=s{2};
z=s{3};
acc=[x y z];
figure(6)
plot(time,acc,'linewidth',lineW)
set(gca,'fontsize',20)
lgd = legend({'X','Y','Z'});
lgd.FontSize = FontS;
title('Estimated WCS readings','FontSize',FontS)
xlabel('Time [s]','FontSize',FontS)
ylabel('Acceleration readings [m/s^2]','FontSize',FontS)
%{
figure(5)
plot(x);
title('Estimated acc X-axis')
xlabel('Estimated acc X-axis')
figure(6)
plot(y);
title('Estimated acc Y-axis')
xlabel('Estimated acc Y-axis')
figure(7)
plot(z);
title('Estimated acc Z-axis')
xlabel('Estimated acc Z-axis')
%}
%% Plot velocity from IMU 
%close all;
fid=fopen('D:\Lagring\Plugg\Examensarbete\Data\images\pile2-2\velocity.txt');
%fid=fopen('C:\Users\Marcus\Desktop\Examensarbete\Data\velocity.txt');
s=textscan(fid,'%f %f %f','headerlines',0);
fclose(fid);
x=s{1};
y=s{2};
z=s{3};
figure(10)
velocity=[x y z];
N=length(s{1});
time = 0:N-1;
time=time*0.01;
plot(time,velocity,'linewidth',lineW)
set(gca,'fontsize',20)
lgd = legend({'X','Y','Z'});
lgd.FontSize = FontS;
title('Estimated velocity','FontSize',FontS)
xlabel('Time [s]','FontSize',FontS)
ylabel('Integrated velocity [m/s]','FontSize',FontS)

%% Plot 3D positions estimated from IMU 
close all;
fid=fopen('D:\Lagring\Plugg\Examensarbete\Data\images\pile2-2\positions.txt');
%fid=fopen('C:\Users\Marcus\Desktop\Examensarbete\Data\positions.txt');
% Nposes=719;
% Nposes=1133;
% Nposes=817;
Nposes=923;

s=textscan(fid,'%f %f %f','headerlines',0);
fclose(fid);
x=s{1};
y=s{2};
z=s{3};
%3d

figure(1)

plot3(x,y,z,'linewidth',lineW); hold on
set(gca,'fontsize',20)
%plot(x,z)
title('Estimated movement by IMU','FontSize',FontS)

xlabel('X [m]','FontSize',FontS)
ylabel('Y [m]','FontSize',FontS)
zlabel('Z [m]','FontSize',FontS)


%start point Blue asterix
plot3([0 0],[0 0],[0 0], 'Marker','o', 'MarkerFaceColor','green', 'MarkerSize',22);
drawnow ;
%endpoint Red circle
plot3(x(end),y(end), z(end),'Marker','o','MarkerFaceColor','red','MarkerSize',22);
% Scale est mark
N = 500/Nposes; %used for scale computation
index = floor(N*length(x));
plot3(x(index),y(index), z(index),'Marker','o','MarkerFaceColor','black','MarkerSize',22);


