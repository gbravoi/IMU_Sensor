% Development of a Knee Brace with Joint Kinematic tracking for ACL 
% Reconstructed Patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2018

classdef kalmanfilter < handle
%This sensor fusion algorithm is based on the paper "Quaternion-Based 
%Kalman Filter for AHRS Using an Adaptive-Step Gradient Descent Algorithm" 
%by Li Wang*, Zheng Zhang, Ping Sun
%http://journals.sagepub.com/doi/full/10.5772/61313
       %% Public properties
    properties (Access = public)
        Quaternion = [1 0 0 0];     % output quaternion describing the Earth relative to the sensor
        P_k=eye(4); %Estimation of error covariance model, start as I
        r_k=7e-6; %The square standard deviation of acceleration in rest (Adjustable parameter).
        q_k=4e-3; %The square standard deviation of angular velocity in rest (Adjustable parameter).
        mu=0.04;%Gradient descent step-size (Adjustable parameter)

    end
methods (Access = public)
        function obj = kalmanfilter(varargin)
            for i = 1:2:nargin
               if  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1}; %starting quaternion
               elseif  strcmp(varargin{i}, 'r_k'), obj.r_k = varargin{i+1};
               elseif  strcmp(varargin{i}, 'q_k'), obj.q_k = varargin{i+1};
               elseif  strcmp(varargin{i}, 'mu'), obj.mu = varargin{i+1};    
                else error('Invalid argument');
                end
            end;
        end
        
        function obj = Update(obj, w, a, T) 
            %angular velocity, acceleration, magnetometer, time step
            
            q = obj.Quaternion; % short name local variable for readability
            R_k=obj.r_k*eye(4); %Covariance matrix observation noise.
            Q_k=obj.q_k*eye(4); %Covariance matrix process noise.
            
            % Normalize accelerometer measurement
            if(norm(a) == 0), return; end	% handle NaN
            a = a / norm(a);	% normalize magnitude
            
            %Kalman observed model.
           H=eye(4);

                f=[ 2*q(2)*q(4)-2*q(1)*q(3)- a(1)
                2*q(3)*q(4) + 2*q(1)*q(2) - a(2)
                q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2 - a(3)];

            Jt=  [ -2*q(3), 2*q(2),  2*q(1)
                 2*q(4), 2*q(1), -2*q(2)
                 -2*q(1),  2*q(4), -2*q(3)
                 2*q(2),  2*q(3),  2*q(4)];
            
            %gradiente(Z_k1)
            df=Jt*f;
            Z_k1=q'-obj.mu*df/norm(df); %%z_(k+1)Value of output observation needed for Kalman filter

            
            %System model       
            wx=[   0   w(3) -w(2)
                -w(3)    0   w(1)
                 w(2) -w(1)    0 ];
             
            Omega=1/2* [0 -w
                        w' wx];
            Phi=eye(4)*(1-1/8*((w(1)*T)^2+(w(2)*T)^2+(w(3)*T)^2))+1/2*Omega*T;


            %Kalman Filter
            %projections
            q_nk1=Phi*q'; 
            P_nk1=Phi*obj.P_k*Phi'+Q_k;
            %Kalman gain
            K_k1=P_nk1*H'*inv(H*P_nk1*H'+R_k);
            %Update
            q=(q_nk1+K_k1*(Z_k1-H*q_nk1))';
            q=q/norm(q);
          
            obj.Quaternion=q;
            obj.P_k=(eye(4)-K_k1*H)*P_nk1;
        end          
end 

end