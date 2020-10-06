% IMU sensor fusion algorithm for monitoring knee kinematics in ACL reconstructed patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2019

close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal


%%  Analysis Data
camera_file = '../data/Example.mat';
imu_file = '../data/Example.txt';
titulo='Double Leg Squats';
[ma_time, ma_kflex] = motion_analysis_validation(camera_file);
[imu_time, kalman_kflex, proposed_kflex] = IMU_analysis_validation(imu_file);

%Manually synchronization
cut=round(23.68/(imu_time(2)-imu_time(1)));
fin= round(9.9/(imu_time(2)-imu_time(1)))+cut;
cut2=round((0.5)/(ma_time(2)-ma_time(1)));
fin2= round((9.4)/(ma_time(2)-ma_time(1)))+cut2;
imu_time=imu_time(cut:fin)-imu_time(cut);
ma_time=ma_time(cut2:fin2)-ma_time(cut2);
proposed_kflex=proposed_kflex(cut:fin);
kalman_kflex=kalman_kflex(cut:fin);
ma_kflex=ma_kflex(cut2:fin2);


%% deg. offset of the imu sensors (if any)
offset=6;
kalman_kflex=kalman_kflex-offset;
proposed_kflex=proposed_kflex-offset;

%% Statistical comparison
L=fin-cut+1;
M1=zeros(L,5);
k=1;
j=2;
for i=1:L-1
   while ma_time(j)<imu_time(i)
       j=j+1;
       if j>length(ma_time)
           break;
       end
   end
   v1=(ma_kflex(j)+ma_kflex(j-1))/2;  
   if (~isnan(v1)) %only write non NaN rows
   M1(k,1)=imu_time(i);
   M1(k,2)=ma_time(j);
   M1(k,3)=proposed_kflex(i); %Proposed algorithm angle
   M1(k,4)=kalman_kflex(i); %Kalman-filter angle
   M1(k,5)=v1; %MOCAP reference angle
   k=k+1;
   end
end
%delete zero rows
M1=M1(1:k-1,:);


%proposed statistics
[proposed_RMSD,proposed_NRMSD]=NRMSD(M1(:,5),M1(:,3));
proposed_RMSD
proposed_CCC= f_CCC([M1(:,5),M1(:,3)],0.1); %alpha is not used
proposed_CCC=proposed_CCC{1,1}.pearsonCorrCoeff
proposed_ICC= f_ICC([M1(:,5),M1(:,3)],0.1);
proposed_ICC21=proposed_ICC{2}.est
proposed_ICC31=proposed_ICC{3}.est

%kalman statistics
[kalman_RMSD, kalman_NRMSD]=NRMSD(M1(:,5),M1(:,4));
kalman_RMSD
kalman_CCC= f_CCC([M1(:,5),M1(:,4)],0.1); %alpha is not used
kalman_CCC=kalman_CCC{1,1}.pearsonCorrCoeff
kalman_ICC= f_ICC([M1(:,5),M1(:,4)],0.1);
kalman_ICC21=kalman_ICC{2}.est
kalman_ICC31=kalman_ICC{3}.est

%% Graphs
figure()
hold on
plot(ma_time, ma_kflex, 'b', 'LineWidth', 1.2);
plot(imu_time, kalman_kflex, 'g', 'LineWidth', 1.1);
plot(imu_time, proposed_kflex, 'r', 'LineWidth', 1.1);
title(strcat(titulo));
ylabel('Knee Flexion/Extension Angle');
xlabel('Time (sec)');
legend('Motion Camera', 'Kalman filter', 'Proposed filter' );
grid on
hold off
saveas(gcf, '../results/figure_1.png');

