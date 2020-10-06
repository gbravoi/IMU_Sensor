% IMU sensor fusion algorithm for monitoring knee kinematics in ACL reconstructed patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2019

function [imu_time, kalman_kflex, proposed_kflex] = IMU_analysis_validation(FileName)
%IMU_analysis_validation computes knee flexion and extension from two IMU
%sensors data, one in the thigh and one in the shank, using a proposed
%sensor fusion algorithm and a Kalman-filter based algorithm.

  f1=opfile('Name',FileName);
  f1.Load;
  L=length(f1.Millis);
  time=(f1.Millis-ones(L,1)*f1.Millis(1))/1000;%start in time 0
  imu_time = time(1:L-1);
  raw_w1=f1.raw_w1;
  raw_a1=f1.raw_a1;
  raw_w2=f1.raw_w2;
  raw_a2=f1.raw_a2;
  

  %% transform rawdata to physical value
  w1=zeros(L,3);
  a1=zeros(L,3);
  w2=zeros(L,3);
  a2=zeros(L,3);
  
  L=length(raw_w1);

  %Angular velocity in radians. phase shift mean=value at rest
  w1(:,1)=0.00875*pi/180*(raw_w1(:,1)-ones(L,1)*431);
  w1(:,2)=0.00875*pi/180*(raw_w1(:,2)+ones(L,1)*216);
  w1(:,3)=-0.00875*pi/180*(raw_w1(:,3)-ones(L,1)*94);

  w2(:,1)=0.00875*pi/180*(raw_w2(:,1)-ones(L,1)*549);
  w2(:,2)=0.00875*pi/180*(raw_w2(:,2)-ones(L,1)*37);
  w2(:,3)=-0.00875*pi/180*(raw_w2(:,3)+ones(L,1)*9);

  %Acceleration in g
  a1(:,1)=-0.000061*raw_a1(:,1);
  a1(:,2)=-0.000061*raw_a1(:,2);
  a1(:,3)=0.000061*raw_a1(:,3);

  a2(:,1)=-0.000061*raw_a2(:,1);
  a2(:,2)=-0.000061*raw_a2(:,2);
  a2(:,3)=0.000061*raw_a2(:,3);

  normaa1=zeros(L);
  normaa2=zeros(L);

  for i=1:L-1
   normaa1(i)=norm(a1(i,:));
   normaa2(i)=norm(a2(i,:));
  end


  %% Proposed filter
    qa_p = zeros(L-1, 4);
    qb_p = zeros(L-1, 4);
    
    % Compute estimated initial quaternion. 
    t=acos(dot(a1(1,:),[0 0 1]));
    V=cross(a1(1,:),[0 0 1]);
    qa_p(1, :)=[cos(t/2) sin(t/2)*V/norm(V)];

    t=acos(dot(a2(1,:),[0 0 1]));
    V=cross(a2(1,:),[0 0 1]);
    qb_p(1, :)=[cos(t/2) sin(t/2)*V/norm(V)];

    %Filter
    filtro_p =ProposedFilter('Beta', 0.4, 'qa', qa_p(1, :), 'qb', qb_p(1, :));
    for t = 2:L
          deltat=time(t)-time(t-1);
          filtro_p.Update(w1(t,:), a1(t,:), w2(t,:), a2(t,:),deltat);
          qa_p(t-1, :) = filtro_p.Quaternion_a;
          qb_p(t-1, :) = filtro_p.Quaternion_b;
    end

    %%relative quaternion.
    qab_p = zeros(L-1, 4);
    for t = 1:L-1
      qab_p(t, :) = quatmultiply(quatconj(qa_p(t,:)),qb_p(t,:));
    end



  %% Kalman Filter
  KalmanFilter_a = kalmanfilter();
  KalmanFilter_b = kalmanfilter();

  %%Compute sensor A filter
  qa_k = zeros(L-1, 4);
  for t = 2:L
      deltat=time(t)-time(t-1);
      KalmanFilter_a.Update(w1(t,:), a1(t,:),deltat);	
      qa_k(t-1, :) = KalmanFilter_a.Quaternion;
  end

  %%Compute sensor B filter
  qb_k = zeros(L-1, 4);
  for t = 2:L
      deltat=(time(t)-time(t-1));
      KalmanFilter_b.Update(w2(t,:), a2(t,:),deltat);	
      qb_k(t-1, :) = KalmanFilter_b.Quaternion;
  end

  %Relative Quaternion
  qab_k = zeros(L-1, 4);
  for t = 1:L-1
      qab_k(t, :) = quatmultiply(quatconj(qa_k(t,:)),qb_k(t,:));
  end


 
  %% Knee flexio/extension (ZYX from relative orientation)
  eulerZYX_k=quat2euler(qab_k,'ZYX')* (180/pi);
  kalman_kflex = -eulerZYX_k(:,3);
    
  eulerZYX_p=quat2euler(qab_p,'ZYX')* (180/pi);
  proposed_kflex = -eulerZYX_p(:,3);
  

end