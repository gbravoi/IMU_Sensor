% Development of a Knee Brace with Joint Kinematic tracking for ACL 
% Reconstructed Patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2018

classdef opfile < handle
%Transform IMU data on a .txt file to matlab.    
%% Public properties
    properties (Access = public)
        filename ='';     
        Millis;
        raw_w1;
        raw_a1;
        raw_m1;
        raw_w2;
        raw_a2;
        raw_m2;
    end
methods (Access = public)
            function obj = opfile(varargin)
            for i = 1:2:nargin
               if  strcmp(varargin{i}, 'Name'), obj.filename = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        function obj = Load(obj)
          
            A = importdata(obj.filename,',',1);

            obj.Millis=A.data(:, 1);
            obj.raw_w1=A.data(:, 2:4);
            obj.raw_a1=A.data(:, 5:7);
            obj.raw_m1=A.data(:, 8:10);
            obj.raw_w2=A.data(:, 11:13);
            obj.raw_a2=A.data(:, 14:16);
            obj.raw_m2=A.data(:, 17:19);
        end
end
end


