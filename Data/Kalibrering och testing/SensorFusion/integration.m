%time = load('D:\Users\Desktop\time.txt');
close all;


fid=fopen('D:\Lagring\Plugg\Examensarbete\Data\acceleration.txt');
%fid=fopen('C:\Users\Marcus\Desktop\Examensarbete\Data\acceleration.txt');

s=textscan(fid,'%f %f %f','headerlines',0);
fclose(fid);
x=s{1};
N=length(s{1});
time = 0:N-1;
time=time*0.01;

y=s{2};
z=s{3};
acc=[x y z];

%% Design High Pass Filter

fs = 8000; % Sampling Rate

fc = 0.1/30; % Cut off Frequency

order = 6; % 6th Order Filter

%% Filter Acceleration Signals

[b1 a1] = butter(order,fc,'high');

accf=filtfilt(b1,a1,acc);
%accf=acc;
figure (2)
plot(time,accf,'r'); hold on
plot(time,acc)
title('Estimated acceleration by IMU')
xlabel('Time (sec)')

ylabel('Acceleration (m/sec^2)')

%% First Integration (Acceleration - Veloicty)

velocity=cumtrapz(time,accf);

figure (3)

plot(time,velocity)
title('Estimated velocity by IMU')
xlabel('Time (sec)')

ylabel('Velocity (m/sec)')

%% Filter Veloicty Signals

[b2 a2] = butter(order,fc,'high');

velf = filtfilt(b2,a2,velocity);
%velf=velocity;
%% Second Integration (Velocity - Displacement)

Displacement=cumtrapz(time, velf);

figure(4)

plot(time,Displacement)
title('Estimated displacement by IMU')
xlabel('Time (sec)')

ylabel('Displacement (m)')

%3d
Displacement = cumsum(Displacement,1);
figure(1)
plot3(Displacement(:,1),Displacement(:,2),Displacement(:,3));
%plot(x,z)
title('Estimated movement by IMU')
xlabel('X')
ylabel('Y')
zlabel('Z')

hold on;
%start point Blue asterix
plot3([0 0],[0 0],[0 0], 'Marker','*', 'LineStyle','none', 'MarkerFaceColor',[1 0 0], 'MarkerSize',10);
drawnow ;
%endpoint Red circle
plot3(Displacement(end,1),Displacement(end,2), Displacement(end,3),'Marker','o','LineStyle','none','MarkerFaceColor',[1 0 0],'MarkerSize',10);

