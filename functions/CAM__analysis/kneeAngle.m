% Development of a Knee Brace with Joint Kinematic tracking for ACL 
% Reconstructed Patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2018

function angle = kneeAngle(p1,p2,p3,p4,p5)
%Knee flexion and extension angle
%p1 thigh lateral
%p2 and p3 knee points
%p4 p5 ankle points


%normal plane thigh-knee (unitary vector)
n1 =plane_normal(p1,p2,p3);

%normal plane knee-ankle (unitary vector)
n2 = plane_normal(p2,p3,(p4+p5)/2);

%angle between both planes=knee flexion/extension angle
angle=180-acos(dot(n1,n2))*180/pi;
end