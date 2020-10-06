% Development of a Knee Brace with Joint Kinematic tracking for ACL 
% Reconstructed Patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2018

function vector = vector2points(p1,p2)
%vector from point 1 to 2

vector=[p1(1)-p2(1) p1(2)-p2(2) p1(3)-p2(3)];
end