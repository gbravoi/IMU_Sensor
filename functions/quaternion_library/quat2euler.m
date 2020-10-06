% Development of a Knee Brace with Joint Kinematic tracking for ACL 
% Reconstructed Patients
% Gabriela Bravo-Illanes, Ryan Halvorson, Robert Peter Matthew, Benjamin Ma, and Ruzena Bajcsy
% EECS, UC Berkeley and Department of Orthopaedic Surgery,UCSF.
% 2018

function euler = quat2euler(q,type)

if strcmp(type,'ZYX')
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)+q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)+q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;

    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -asin(R(3,1,:));
    psi = atan2(R(2,1,:), R(1,1,:) );

    euler = [ psi(1,:)' theta(1,:)'  phi(1,:)']; % Third, second and firs rotation
end
end

