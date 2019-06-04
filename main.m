%clc; 
global e a u R startIndex estimationInterval motionStartIndex includeRealConfiguration; 

e = 6.69437999013e-3; % squared eccentricity
a = 6378137; % big axle (in meters)
u = 0.72921158553e-4; % angular velocity of Earth
R = 6370000; % earth radius 
startIndex = 1;   % initial index 
estimationInterval = 12000;  % start interval (120s * 100Hz)
motionStartIndex = estimationInterval + 1;   % first index pf data, when motion starts
includeRealConfiguration = true; % include optional plots and calculations

%%%%%%%%%%%%%
%preparing trajectory data as matlab table
trj_prp = fileread('./insdemo_v2/trj.txt');
trj_prp = regexprep(trj_prp,'% ','');
trj_txt = fopen('./insdemo_v2/trj.txt', 'wt');
fprintf(trj_txt, '%s', trj_prp);
fclose(trj_txt);

%reading as table
trj_data = readtable('./insdemo_v2/trj.txt','Format','%f%f%f%f%f%f%f%f%f%f','Delimiter',' ','MultipleDelimsAsOne',1); % trajectory

h0 = trj_data{1,'alt_m_'}; % initial height

%%%%%%%%%%%%%
%preparing sensors data as matlab table
imu_prp = fileread('./insdemo_v2/imu.txt');
imu_prp = regexprep(imu_prp,'% ','');
imu_txt = fopen('./insdemo_v2/imu.txt', 'wt');
fprintf(imu_txt, '%s', imu_prp);
fclose(imu_txt);

%deserialize input data (accelerometers and angular velocity measures)
imu_data = readtable('./insdemo_v2/imu.txt','Format','%f%f%f%f%f%f%f','Delimiter',' ','MultipleDelimsAsOne',1); 

t = imu_data{:, 'time_s_'};

%angular velocities
omega1 = deg2rad(imu_data{:, 'wz1_dps_'});  
omega2 = deg2rad(imu_data{:, 'wz2_dps_'});  
omega3 = deg2rad(imu_data{:, 'wz3_dps_'});  
    
%forces
z1 = imu_data{:, 'fz1_mps2_'}; 
z2 = imu_data{:, 'fz2_mps2_'};
z3 = imu_data{:, 'fz3_mps2_'};

%DISTURBANCES
disturb1 = ones(size(z1));
disturb2 = ones(size(z1));
disturb3 = ones(size(z1));

% get mean on estimation interval
fz = mean(imu_data{1 : estimationInterval, 5 : 7});
disp(fz);
uz = mean(imu_data{1 : estimationInterval, 2 : 4});  
disp(uz);

fz_d = mean(imu_data{1 : estimationInterval, 5 : 7}) + [mean(disturb1(1 : estimationInterval)) mean(disturb2(1 : estimationInterval)) mean(disturb3(1 : estimationInterval))];

%deserialize correct values of coordinates and velocities
t2 = trj_data{:,'time_s_'}; % time

lat = deg2rad(trj_data{:,'lat_deg_'});  
long = deg2rad(trj_data{:,'lon_deg_'});
h = trj_data{:,'alt_m_'}; 

Ve = trj_data{:,'Ve_mps_'}; % velocity to the East
Vn = trj_data{:,'Vn_mps_'}; % velocity to the North
Vh = trj_data{:,'Vup_mps_'}; % velocity with respect to surface normal

% Krylov's angles
head = deg2rad(trj_data{:,'heading_deg_'});  
pitch = deg2rad(trj_data{:,'pitch_deg_'});   
roll = deg2rad(trj_data{:,'roll_deg_'}); 

% Init values of velocities and coordinates
phi0 = lat(startIndex); 
lambda0 = long(startIndex);
Ve0 = Ve(startIndex); 
Vn0 = Vn(startIndex);
Vh0 = Vh(startIndex);

% calculating real free fall acceleration
g = 9.78030*(1 + 0.0053020*(sin(phi0))^2 - 0.0000070*(sin(2*phi0))^2) - 0.00014 - 0.000003086*h0;

%initializing rotation matrix by mean sensors value
L0col1 = cross(uz, fz)/norm(cross(uz, fz));
L0col2 = cross(fz, cross(uz, fz))/norm(cross(fz, cross(uz, fz)));
L0col3 = fz/norm(fz);
L0 = [L0col1' L0col2' L0col3']; 

%DISTURBED
L0col1_d = cross(uz, fz_d)/norm(cross(uz, fz_d));
L0col2_d = cross(fz_d, cross(uz, fz_d))/norm(cross(fz_d, cross(uz, fz_d)));
L0col3_d = fz_d/norm(fz_d);
L0_d = [L0col1_d' L0col2_d' L0col3_d'];

%code for checking???
%re-calculating 3rd column of L by true values of angles
% properTeta  = 0;
% properGamma = 0;
% properPsi = -1.483529864195180;
%L(:, 3) = fz / g;
%{
L(:, 3) = [0 0 1];
sinTeta = L(2, 3);
properTeta = asin(sinTeta);
cosGamma = L(3, 3) / cos(properTeta);
properGamma = -abs(acos(cosGamma));
%}
%{
L( :, 3) = [
    sin(properTeta);
    cos(properTeta)*cos(properGamma);
    -cos(properTeta)*sin(properTeta)
    ];

L(:, 2) = [
    (uz(1)-L(1,3)*norm(uz)*sin(phi0))/(cos(phi0)*norm(uz));
    (uz(2)-L(2,3)*norm(uz)*sin(phi0))/(cos(phi0)*norm(uz));
    (uz(3)-L(3,3)*norm(uz)*sin(phi0))/(cos(phi0)*norm(uz))
    ];

cosPsi = L(2, 2) / cos(properTeta);
properPsi = -acos(cosPsi);

L(:, 2) = [
    cos(properPsi)*cos(properTeta);
    -sin(properTeta)*cos(properPsi)*cos(properGamma) - sin(properPsi)*sin(properGamma);
    sin(properTeta)*cos(properPsi)*sin(properGamma) - sin(properPsi)*cos(properGamma)
    ];

L(:, 1) = [
    sin(properPsi)*cos(properTeta);
    cos(properPsi)*sin(properGamma) - sin(properPsi)*sin(properTeta)*cos(properGamma);
    cos(properPsi)*cos(properGamma) + sin(properPsi)*sin(properTeta)*sin(properGamma)
    ]; 
%}

%computing Krylov angles out of matrix
properPsi = atan2(L0(1,1),L0(1,2));
properTeta = atan2(L0(1,3),sqrt(L0(1,1)^2+L0(1,2)^2));
properGamma = atan2(-L0(3,3),L0(2,3));

disp([properPsi, properTeta, properGamma])

% start 2nd stage of solution
% initializing matrices 
Az = L0; 
Ax = eye(3);

Az_d = L0_d; 
Ax_d = eye(3);

%creating arrays for state variables
lambda = zeros(size(z1));
phi = zeros(size(z1));
h_s = zeros(size(z1));
Ve_s = zeros(size(z1));
Vn_s = zeros(size(z1));
Vh_s = zeros(size(z1));
%psi = zeros(size(z1));
%gam = zeros(size(z1));
%tet = zeros(size(z1));

%DISTURBED
lambda_d = zeros(size(z1));
phi_d = zeros(size(z1));
h_s_d = zeros(size(z1));
Ve_s_d = zeros(size(z1));
Vn_s_d = zeros(size(z1));
Vh_s_d = zeros(size(z1));

%creating arrays for matrix component difference
L11t = zeros(size(z1));
L12t = zeros(size(z1));
L13t = zeros(size(z1));
L21t = zeros(size(z1));
L22t = zeros(size(z1));
L23t = zeros(size(z1));
L31t = zeros(size(z1));
L32t = zeros(size(z1));
L33t = zeros(size(z1));

%initializing variables at rest
for i = 1 : motionStartIndex   
   lambda(i) = lambda0;
   phi(i) = phi0;
   h_s(i) = h0;
   Ve_s(i) = Ve0;
   Vn_s(i) = Vn0;
   Vh_s(i) = Vh0;
   %psi(i) = properPsi;
   %gam(i) = properGamma;
   %tet(i) = properTeta;
end

%initializing arrays for differences in integrated and exact velocity values 
%{
if includeRealConfiguration == true
    deltaTDotV1 = zeros(size(Ve) - 3);
    deltaTDotV2 = zeros(size(Ve) - 3);
    deltaTDotV3 = zeros(size(Ve) - 3);
end
%}

%INTEGRATION ALGORITHM
%dec size(t) to avoid exceptions in getting k+1 element
for k = motionStartIndex : (size(t)-1)
    
    deltaT = t(k+1) - t(k);

    omega_z = sqrt(omega1(k)^2 + omega2(k)^2 + omega3(k)^2);
    
    c1 = sin(omega_z*deltaT) / omega_z;
    c2 = (1-cos(omega_z*deltaT)) / (omega_z^2) ;

    Omega_z = [0 ,            omega3(k),      - omega2(k);...  
            -omega3(k),     0 ,               omega1(k);...
             omega2(k),      - omega1(k) ,           0];
    
    % A_i+1  recurrent method
    % using previous Az calculated 1 iteration before
    Az = (eye(3) + c1*Omega_z+ c2* Omega_z^2) * Az;  
    
    %DISTURBED
    Az_d = (eye(3) + c1*Omega_z+ c2* Omega_z^2) * Az_d;  

    % Curve radiuses (east and north)
    Re = a / sqrt(1 - e*(sin(phi(k)))^2);
    Rn = a * (1 - e) / (1 - e * (sin(phi(k)))^2)^(3/2);
    
    %DISTURBED
    Re_d = a / sqrt(1 - e*(sin(phi_d(k)))^2);
    Rn_d = a * (1 - e) / (1 - e * (sin(phi_d(k)))^2)^(3/2);
    
    % calculating current free fall acceleration
    g = 9.78030*(1 + 0.0053020 * (sin(phi(k)))^2 - 0.0000070*(sin(2*phi(k)))^2) - 0.00014 - 0.000003086*h_s(k);
    %DISTURBED
    g_d = 9.78030*(1 + 0.0053020 * (sin(phi_d(k)))^2 - 0.0000070*(sin(2*phi_d(k)))^2) - 0.00014 - 0.000003086*h_s_d(k);
    
    % angular velocity with respect to geographical axes
    uE = 0;  
    uN = u * cos(phi(k));
    uH = u * sin(phi(k));
    %DISTURBED
    uE_d = 0;  
    uN_d = u * cos(phi_d(k));
    uH_d = u * sin(phi_d(k));

    %components of geographical repper angular velocity with respect to Earth
    omegaE = -Vn_s(k) / (Rn + h_s(k));  
    omegaN = Ve_s(k) / (Re + h_s(k));   
    omegaH = omegaN * tan(phi(k));  
    %DISTURBED
    omegaE_d = -Vn_s_d(k) / (Rn_d + h_s_d(k));  
    omegaN_d = Ve_s_d(k) / (Re_d + h_s_d(k));   
    omegaH_d = omegaN_d * tan(phi_d(k)); 

    % Filling matrix 
    Omega_x = [0 ,            omegaH+uH,      - omegaN-uN;...
            -omegaH-uH,     0 ,               omegaE+uE;...
             omegaN+uN,     -omegaE-uE ,             0];
    %DISTURBED
    Omega_x_d = [0 ,            omegaH_d+uH_d,      - omegaN_d-uN_d;...
            -omegaH_d-uH_d,     0 ,               omegaE_d+uE_d;...
             omegaN_d+uN_d,     -omegaE_d-uE_d ,             0];

    % calculating omega + ux (defined above)
    omega_x = norm([omegaE+uE, omegaN+uN, omegaH+uH]);
    %DISTURBED
    omega_x_d = norm([omegaE_d+uE_d, omegaN_d+uN_d, omegaH_d+uH_d]);
 
    c1 = sin(omega_x*deltaT) / omega_x;
    c2 = (1 - cos(omega_x*deltaT)) / (omega_x^2);
    %DISTURBED
    c1_d = sin(omega_x_d*deltaT) / omega_x_d;
    c2_d = (1 - cos(omega_x_d*deltaT)) / (omega_x_d^2);
    
    % A_i+1  recurrent method
    % using previous Ax calculated 1 iteration before
    Ax = (eye(3) + c1*Omega_x+ c2* Omega_x^2) * Ax;
    %DISTURBED
    Ax_d = (eye(3) + c1_d*Omega_x_d+ c2_d* Omega_x_d^2) * Ax_d;

    % Re-calculating L in new iteration
    % Has to be always ortogonal
    L = Az * Ax'; 
    %DISTURBED
    L_d = Az_d * Ax_d'; 
    
    %checking ortogonality of rotation matrix
    psi1 = head(k);
    teta1 = pitch(k);
    gamma1 = roll(k);
    L_true = [
        cos(teta1)*sin(psi1), cos(teta1)*cos(psi1), sin(teta1);...
        -sin(teta1)*sin(psi1)*cos(gamma1)+cos(psi1)*sin(gamma1), -sin(teta1)*cos(psi1)*cos(gamma1)-sin(psi1)*sin(gamma1), cos(teta1)*cos(gamma1);...
        sin(teta1)*sin(psi1)*sin
        (gamma1)+cos(psi1)*cos(gamma1), sin(teta1)*cos(psi1)*sin(gamma1)-sin(psi1)*cos(gamma1), -cos(teta1)*sin(gamma1)
        ];
    
    L_diff = L' * L_true-eye(3);
    L11t(k) = L_diff(1,1);
    L22t(k) = L_diff(2,2);
    L21t(k) = L_diff(2,1);
    L13t(k) = L_diff(1,3);
    L12t(k) = L_diff(1,2);
    L23t(k) = L_diff(2,3);
    L31t(k) = L_diff(3,1);
    L32t(k) = L_diff(3,2);
    L33t(k) = L_diff(3,3);

    %forces in inertial coordinate system
    fz = [z1(k); z2(k); z3(k)];
    fy = L' * fz;  
    %DISTURBED
    fz_d = [z1(k)+disturb1(k); z2(k)+disturb2(k); z3(k)+disturb3(k)];
    fy_d = L_d' * fz_d;  

    %equations for acceleration i geographical coordinates
    dotVe =  (omegaH + 2*uH)*Vn_s(k) - (omegaN + 2*uN)*Vh_s(k) + fy(1,1);
    dotVn = -(omegaH + 2*uH)*Ve_s(k) + (omegaE + 2*uE)*Vh_s(k) + fy(2,1);
    dotVh =  (omegaN + 2*uN)*Ve_s(k) - (omegaE + 2*uE)*Vn_s(k) + fy(3,1) - g;
    %DISTURBED
    dotVe_d =  (omegaH_d + 2*uH_d)*Vn_s_d(k) - (omegaN_d + 2*uN_d)*Vh_s_d(k) + fy_d(1,1);
    dotVn_d = -(omegaH_d + 2*uH_d)*Ve_s_d(k) + (omegaE_d + 2*uE_d)*Vh_s_d(k) + fy_d(2,1);
    dotVh_d =  (omegaN_d + 2*uN_d)*Ve_s_d(k) - (omegaE_d + 2*uE_d)*Vn_s_d(k) + fy_d(3,1) - g_d;

    %{
    for indexVe = 1 : size(Ve)
        % Optional calculations for plots
        % k - index of outer loop
        if includeRealConfiguration == true
            if t(k) == t2(indexVe)
                teta2 = pitch(indexVe);
                gamma2 = roll(indexVe);
                psi2 = head(indexVe);
                L1(:, 3) = [-cos(teta2) * sin(gamma2); sin(teta2); cos(teta2)*cos(gamma2)];
                L1(:, 2) = [sin(psi2)*cos(gamma2) + cos(psi2)*sin(teta2)*sin(gamma2); cos(psi2)*cos(teta2); sin(psi2)*sin(gamma2) - cos(psi2)*sin(teta2)*cos(gamma2)];
                L1(:, 1) = [cos(psi2)*cos(gamma2) - sin(psi2)*sin(teta2)*sin(gamma2); -sin(psi2)*cos(teta2); cos(psi2)*sin(gamma2) + sin(psi2)*sin(teta2)*cos(gamma2)]; 
             
                fy1 = L1 * fz;
                
                dotVe1 = (omegaH + 2*uH)*Vn(indexVe) - (omegaN + 2*uN)*Vh(indexVe) + fy1(1,1);
                dotVn1 = -(omegaH + 2*uH)*Ve(indexVe) + (omegaE + 2*uE)*Vh(indexVe) + fy1(2,1);
                dotVh1 = (omegaN + 2*uN)*Ve(indexVe) - (omegaE + 2*uE)*Vn(indexVe) + fy1(3,1) - g;

                deltaTDotV1(indexVe) = (dotVe - dotVe1);
                deltaTDotV2(indexVe) = (dotVn - dotVn1);
                deltaTDotV3(indexVe) = (dotVh - dotVh1);
            end
         end
    end
    %}

    % filling last delta's of velocities
    Ve_s(k+1)=  Ve_s(k) + deltaT*dotVe;  
    Vn_s(k+1)=  Vn_s(k) + deltaT*dotVn;  
    Vh_s(k+1)=  Vh_s(k) + deltaT*dotVh;
    %DISTURBED
    Ve_s_d(k+1)=  Ve_s_d(k) + deltaT*dotVe_d;  
    Vn_s_d(k+1)=  Vn_s_d(k) + deltaT*dotVn_d;  
    Vh_s_d(k+1)=  Vh_s_d(k) + deltaT*dotVh_d;
    
    % calculating new values of geographical coordinates
    lambda(k+1) = lambda(k) + deltaT*Ve_s(k)/((Re+h_s(k))*cos(phi(k)));
    phi(k+1)= phi(k) + deltaT*Vn_s(k)/(Rn+h_s(k));
    h_s(k+1) = h_s(k) + deltaT*Vh_s(k);
    %DISTURBED
    lambda_d(k+1) = lambda_d(k) + deltaT*Ve_s_d(k)/((Re_d+h_s_d(k))*cos(phi_d(k)));
    phi_d(k+1)= phi_d(k) + deltaT*Vn_s_d(k)/(Rn_d+h_s_d(k));
    h_s_d(k+1) = h_s_d(k) + deltaT*Vh_s_d(k);

end

%transform from geographical coordinates to inertial xyz
x = (h + R*sqrt(1 - (e*sin(lat)).^2)).*cos(lat).*cos(long);
y = (h + R*sqrt(1 - (e*sin(lat)).^2)).*cos(lat).*sin(long);
z = (h + R*sqrt(1 - (e*sin(lat)).^2)).*sin(lat);

%transform from geographical coordinates to inertial xyz for sensor data
x_s = (h_s + R*sqrt(1 - (e*sin(phi)).^2)).*cos(phi).*cos(lambda);
y_s = (h_s + R*sqrt(1 - (e*sin(phi)).^2)).*cos(phi).*sin(lambda);
z_s = (h_s + R*sqrt(1 - (e*sin(phi)).^2)).*sin(phi);

x_s_d = (h_s_d + R*sqrt(1 - (e*sin(phi_d)).^2)).*cos(phi_d).*cos(lambda_d);
y_s_d = (h_s_d + R*sqrt(1 - (e*sin(phi_d)).^2)).*cos(phi_d).*sin(lambda_d);
z_s_d = (h_s_d + R*sqrt(1 - (e*sin(phi_d)).^2)).*sin(phi_d);

dx = x - x_s;
dy = y - y_s;
dz = z - z_s;

%disturbance change
dx_d = x_s - x_s_d;
dy_d = y_s - y_s_d;
dz_d = z_s - z_s_d;

%transformations from inertial coordinate system to probe coordinate system
A1 = [[cos(long),        sin(long),        zeros(size(long))],...
      [-sin(long),       cos(long),        zeros(size(long))],...
      [zeros(size(long)),zeros(size(long)),ones(size(long))]];
A2 = [[cos(lat),        zeros(size(lat)), sin(lat)],...
      [zeros(size(lat)), ones(size(lat)), zeros(size(lat))],...
      [-sin(lat),        zeros(size(lat)),cos(lat)]];
A3 = [[cos(pitch).*sin(head),                                    cos(pitch).*cos(head),                                    sin(pitch)],...
      [-sin(pitch).*sin(head).*cos(roll)+cos(head).*sin(roll),-sin(pitch).*cos(head).*cos(roll)-sin(head).*sin(roll),cos(pitch).*cos(roll)],...
      [sin(pitch).*sin(head).*sin(roll)+cos(head).*cos(roll), sin(pitch).*cos(head).*sin(roll)-sin(head).*cos(roll), -cos(pitch).*sin(roll)]];

sz = size(long);
dr = zeros(sz(1),3);
dr_d = zeros(sz(1),3);
for i1 = 1:size(long)
    dr(i1,:) = reshape(A3(i1,:),[3,3])*reshape(A2(i1,:),[3,3])*reshape(A1(i1,:),[3,3])*[dx(i1);dy(i1);dz(i1)]; 
    dr_d(i1,:) = reshape(A3(i1,:),[3,3])*reshape(A2(i1,:),[3,3])*reshape(A1(i1,:),[3,3])*[dx_d(i1);dy_d(i1);dz_d(i1)];
end
dr = dr';
dr_d = dr_d';

%%%%%%%%%%%%
% SOMETHING WEIRD
% transform to Cartesian coordinates and move to (0,0)
% h - already good
%{
x_0 = (R + h(1)) * long(1);
y_0 = (R + h(1)) * log(tan(pi/4 + 0.5*lat(1)));
x = zeros(size(long));
y = zeros(size(long));
for i1 = 1 : size(long) 
    x(i1) = (R + h(i1)) * long(i1) - x_0;
    y(i1) = (R + h(i1)) * log(tan(pi/4 + 0.5*lat(i1))) - y_0;
end

% transform to Cartesian coordinates and move to (0,0)
x_1_0 = (R + h_s(1)) * lambda(1);
y_1_0 = (R + h_s(1)) * log(tan(pi/4 + 0.5*phi(1)));
x1 = zeros(size(lambda'));
y1 = zeros(size(lambda'));
h1 = zeros(size(lambda'));
for i1 = 1 : size(lambda')
        x1(i1) = (R + h_s(i1)) * lambda(i1) - x_1_0;
        y1(i1) = (R + h_s(i1)) * log(tan(pi/4 + 0.5*phi(i1))) - y_1_0;
        h1(i1) = h_s(i1);
end    
%}


% Printig trajectories - true and calculated
figure('Name', 'Trajectories');
clf;
plot3(x,y,h,'k','LineWidth' ,1);
hold on;
plot3(x_s,y_s,h_s,'r','LineWidth' ,1);
grid on;

figure('Name', 'Errors in MZ');
clf;
plot3(dr(1,:),dr(2,:),dr(3,:),'k','LineWidth' ,1);
grid on;

figure('Name', 'Disturbances in MZ');
clf;
plot3(dr_d(1,:),dr_d(2,:),dr_d(3,:),'k','LineWidth' ,1);
grid on;

figure('Name', 'Error in Mz1');
clf;
plot(t, dr(1,:),'k','LineWidth' ,1);
grid on;
figure('Name', 'Error in Mz2');
clf;
plot(t, dr(2,:),'k','LineWidth' ,1);
grid on;
figure('Name', 'Error in Mz3');
clf;
plot(t, dr(3,:),'k','LineWidth' ,1);
grid on;

%printing rotation matrix component error
%{
figure('Name','12, 21, 13, 31, 23, 32');
clf;
plot(t,L12t,'r',t,L21t,'g',t,L13t,'b',t,L31t,'y',t,L23t,'m',t,L32t,'k');
%}
%%%%%%%%%%%%
% Calculating error beetween positions in the end of motion
%{ 
r1 = [  (R + h_s(k)) * sin(phi(k)) * cos(lambda(k))
        (R + h_s(k)) * sin(phi(k)) * sin(lambda(k))
        (R + h_s(k)) * cos(phi(k))];
    
r2 = [  (R + h(k)) * sin(lat(k)) * cos(long(k))
        (R + h(k)) * sin(lat(k)) * sin(long(k))
        (R + h(k)) * cos(k)];
%}
    
% Distance beetween endpoints
r1 = [x(end) y(end) h(end)];
r2 = [x_s(end) y_s(end) h_s(end)];

r = norm(r1-r2);
disp(r);

%%%%%%%%%%
%don't know why 901, got to change
%{
if includeRealConfiguration == true
    deltaTV1 = zeros(901, 1);
    deltaTV2 = zeros(901, 1);
    deltaTV3 = zeros(901, 1);
    
    % calculating velocity differences in last time (optional)
    dotVe1 =  (omegaH + 2*uH)*Vn(end) - (omegaN + 2*uN)*Vh(end) + fy(1,1);
    dotVn1 = -(omegaH + 2*uH)*Ve(end) + (omegaE + 2*uE)*Vh(end) + fy(2,1);
    dotVh1 =  (omegaN + 2*uN)*Ve(end) - (omegaE + 2*uE)*Vn(end) + fy(3,1) - g;
    deltaTDotV1(end) = (dotVe - dotVe1);
    deltaTDotV2(end) = (dotVn - dotVn1);
    deltaTDotV3(end) = (dotVh - dotVh1);

    % Differnces beetween velocities
    for i = 1 : size(Ve)
        for indexVe = 1 : size(Ve_s)
             if t(indexVe) == t2(i)
                 deltaTV1(i) = (Ve(i) - Ve_s(indexVe));
                 deltaTV2(i) = (Vh(i) - Vh_s(indexVe));
                 deltaTV3(i) = (Vn(i) - Vn_s(indexVe));
             end
        end
    end

    deltaTV1(end) = (Ve(end) - Ve_s(end));
    deltaTV2(end) = (Vh(end) - Vh_s(end));
    deltaTV3(end) = (Vn(end) - Vn_s(end));

    % Printing plots of dotV differnces
    figure('Name', 'Delta dotV east');clf;
    plot(t2(3 : 901), deltaTDotV1(3:901));
    hold on;
    grid on;

    figure('Name', 'Delta dotV height');clf;
    plot(t2(3 : 901), deltaTDotV2(3:901));
    hold on;
    grid on;

    figure('Name', 'Delta dotV north');clf;
    plot(t2(3 : 901), deltaTDotV3(3:901));
    hold on;
    grid on;

    %%%%%%%%%%%
    % Printing plots of V differences
    figure('Name', 'Delta V east');clf;
    plot(t2, deltaTV1);
    hold on;
    grid on;

    figure('Name', 'Delta V height');clf;
    plot(t2, deltaTV2);
    hold on;
    grid on;

    figure('Name', 'Delta V north');clf;
    plot(t2, deltaTV3);
    hold on;
    grid on;

    % Printing velocities 
    figure('Name', 'East Velocity (true and calculated)');clf;
    plot(t2, Ve)
    hold on
    plot(t,Ve_s)
    hold on

    figure('Name', 'Height Velocity (true and calculated)');clf;
    plot(t2, Vh)
    hold on
    plot(t, Vh_s)
    hold on

    figure('Name', 'North Velocity (true and calculated)');clf;
    plot(t2, Vn)
    hold on
    plot(t, Vn_s)
    hold on
    grid on;

end
%}
   
