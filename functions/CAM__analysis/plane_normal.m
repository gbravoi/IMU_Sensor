% Development of a Knee Brace with Joint Kinematic tracking for ACL 
% Reconstructed Patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2018

function normal = plane_normal(p1,p2,p3)
%Plane normal vector from 3 points (non linear)

%determine the vector form point 1 to 2
AB=vector2points(p1,p2);

%determine the vector form point 1 to 3
AC=vector2points(p1,p3);

%normal
n = cross(AB,AC);
normal=n/norm(n);
end