% Development of a Knee Brace with Joint Kinematic tracking for ACL 
% Reconstructed Patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2018

classdef ProposedFilter < handle
% This sensor fusion algorithm minimizes the difference between X axis of 
%both sensors, and each sensor with the gravity alignment
    %% Public properties
    properties (Access = public)
        Quaternion_a = [1 0 0 0];     % output quaternion describing the sensor relative to earth
        Quaternion_b = [1 0 0 0];
        Beta = 0.1;   % algorithm gain
    end
    %% Methods
    methods (Access = public)
        function obj = ProposedFilter(varargin)
            for i = 1:2:nargin
               if  strcmp(varargin{i}, 'qa'), obj.Quaternion_a = varargin{i+1};
               elseif strcmp(varargin{i}, 'qb'), obj.Quaternion_b = varargin{i+1};
               elseif strcmp(varargin{i}, 'Beta'), obj.Beta = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        
        %% Update
        function obj = Update(obj, wa, aa, wb, ab, SamplePeriod)
            qa = obj.Quaternion_a; % short name local variable for readability
            qb = obj.Quaternion_b;
            
           
            % Normalise accelerometer measurement
            if(norm(aa) == 0), return; end	% handle NaN
            aa = aa / norm(aa);	% normalise magnitude
            if(norm(ab) == 0), return; end	% handle NaN
            ab = ab / norm(ab);	% normalise magnitude 
            
            %Gradient decent algorithm corrective step
            F=[ 2*(qa(1)^2 + qa(2)^2  - qb(1)^2 - qb(2)^2)
              2*(qa(1)*qa(4) + qa(2)*qa(3) - qb(1)*qb(4) - qb(2)*qb(3))
              2*(qa(2)*qa(4) - qa(1)*qa(3) + qb(1)*qb(3) - qb(2)*qb(4))
              2*qa(2)*qa(4) - 2*qa(1)*qa(3) - aa(1)
              2*qa(1)*qa(2) + 2*qa(3)*qa(4) - aa(2)
              2*(0.5- qa(2)^2 - qa(3)^2) - aa(3)
              2*qb(2)*qb(4) - 2*qb(1)*qb(3) - ab(1)
              2*qb(1)*qb(2) + 2*qb(3)*qb(4) - ab(2) 
              2*(0.5- qb(2)^2 - qb(3)^2) - ab(3)];

            J=  [  4*qa(1),  2*qa(4), -2*qa(3), -2*qa(3), 2*qa(2),  0,           0,       0,        0
                   4*qa(2),  2*qa(3),  2*qa(4),  2*qa(4), 2*qa(1), -4*qa(2),     0,       0,        0
                   0,        2*qa(2), -2*qa(1), -2*qa(1), 2*qa(4), -4*qa(3),     0,       0,        0
                   0,        2*qa(1),  2*qa(2),  2*qa(2), 2*qa(3),  0,           0,       0,        0
                  -4*qb(1), -2*qb(4),  2*qb(3),      0,     0,      0,          -2*qb(3), 2*qb(2),  0
                  -4*qb(2), -2*qb(3), -2*qb(4),      0,     0,      0,           2*qb(4), 2*qb(1), -4*qb(2)
                   0,       -2*qb(2),  2*qb(1),      0,     0,      0,          -2*qb(1), 2*qb(4), -4*qb(3)
                   0,       -2*qb(1), -2*qb(2),      0,     0,      0,           2*qb(2), 2*qb(3),  0]; %This is J transpose
            
            step = (J*F);
            stepa = step(1:4) / norm(step(1:4));	% normalize step magnitude
            stepb = step(5:8) / norm(step(5:8));	% normalise step magnitude
            

            
            % Compute rate of change of quaternion
            qDota = 0.5 * quatmultiply(qa, [0 wa]) - obj.Beta * stepa';
            qDotb = 0.5 * quatmultiply(qb, [0 wb]) - obj.Beta * stepb';


 
           % Integrate to yield quaternion
            qa = qa + qDota * SamplePeriod;
            qb = qb + qDotb * SamplePeriod;
            
            
            
            obj.Quaternion_a = qa / norm(qa); % normalise quaternion
            obj.Quaternion_b = qb / norm(qb); % normalise quaternion
        end
    end
end