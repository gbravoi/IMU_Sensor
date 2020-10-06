% IMU sensor fusion algorithm for monitoring knee kinematics in ACL reconstructed patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2019

function [ma_time, ma_kflex] = motion_analysis_validation(FileName)
%motion_analysis_validation computes knee flexion and extension from MOCAP 
%Data.


  load(FileName);

  L = length(marker.time);
  time = marker.time-marker.time(1);

  ma_time = time;

  %In the example we only care about left leg
  THI=marker.lTHIL; 
  KNEM=marker.lKNEM;
  KNEL = marker.lKNEL;
  ANKM = marker.lANKM;
  ANKL = marker.lANKL;
  
 
    
  %% Knee flexion and extension angle 
  %computed has the angle between the plane lTHIL-lKNEM-lKNEL y the plane
  %lKNEM-lKNEL- (lANKM+lANKL)/2
   
  ma_kflex = zeros(L,1);
  for t=1:L
  
    ma_kflex(t)=kneeAngle(THI(t,:),KNEM(t,:),KNEL(t,:),ANKM(t,:),ANKL(t,:));
  end
 
end