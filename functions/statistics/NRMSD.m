% Development of a Knee Brace with Joint Kinematic tracking for ACL 
% Reconstructed Patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2018

function [RMSD,NRMSD]=NRMSD(alpha, beta)
%alpha reference
%Betha measurement
%both vectors of the same length
L=length(alpha);
ys=0;
v1=alpha-beta;
for i=1:L
    ys=ys+v1(i)^2;
end
RMSD=sqrt(ys/L);
NRMSD=RMSD/(max(alpha)-min(alpha))*100;
end

