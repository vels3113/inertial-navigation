clc; clear all;
global omega0 e a  u; 
%omega0 =  1.2383e-3; 
%omega0 = omega0^2; % 
e = 6.69437999013e-3; % ??????????????
a = 6378137; % ??????? ??????? ??????? (?????) 
u = 7292115.8553e-11; % ??????? ???????? ????? 
R = 6370000;
% ????????? ??????
data = load('start.dat'); % ??????????
phi = data(1)/180*pi; 
lambda = data(2)/180*pi; 
phi0 = phi; 
lambda0 = lambda; 
h0 = data(3);
course = data(4);

Nvi = 1;   % ??�??? ????????
Nvo = 12000;  % ????? ????????
Ni = 12000;   % ??�??? ????????


data = load('imu.dat');% ?????? ?? imu.dat
    t = data(:,1);
    omega1=deg2rad(data(:,2));  % ?????????? ??????? ????????  
    omega2=deg2rad(data(:,3));  
    omega3=deg2rad(data(:,4));  
    z1 = data(:,5); % ???????? ????????? ?? ??? z1 .. z2
    z2 = data(:,6);
    z3 = data(:,7);
    fz = mean(data(1:Nvo,5:7)) %??�?????? ????????
    uz = mean(data(1:Nvo,2:4))
    ux = [0,u*cos(phi),u*sin(phi)];  
    
data = load('trj.dat');% ?????? ?? traj.dat
t2 = data(:,1); % ?????
% ????????�????? ??????????
lat = deg2rad(data(:,2));  % ??????
long = deg2rad(data(:,3)); % ???????
h = data(:,4); % ??????
Ve = data(:,5); % ????????  Ve - v1, Vn - v2
Vn = data(:,6);
Vh = data(:,7);
% ???? ??????? ??? ????????
head=deg2rad(data(:,8));  % ???????? (???????? ????)
pitch=deg2rad(data(:,9));    % ??????
roll=deg2rad(data(:,10)); % ????

% ?????????? ??�??? ????????
phi0=lat(Nvi); 
ld0=long(Nvi);
Ve0=Ve(Nvi); 
Vn0=Vn(Nvi);
Vh0=Vh(Nvi);


% ??????? ???????? ?????????
% 2*omega0*h0 - [omega, v]
% https://ru.wikipedia.org/wiki/%D0%A3%D1%81%D0%BA%D0%BE%D1%80%D0%B5%D0%BD%D0%B8%D0%B5_%D1%81%D0%B2%D0%BE%D0%B1%D0%BE%D0%B4%D0%BD%D0%BE%D0%B3%D0%BE_%D0%BF%D0%B0%D0%B4%D0%B5%D0%BD%D0%B8%D1%8F
 g = 9.78030*(1+0.0053020*(sin(phi0))^2-0.0000070*(sin(2*phi0))^2) - 0.00014 - 0.000003086*h0 
% g = 9.8155;
 %g = 9.78030*(1 - 2*h0/a + (3/4)*e*(sin(phi0))^2);
 
% ??????? ??????????
% L=zeros(3,3);
% L(:,3)=fz/g;
% L(:,2)=[(uz(1)-L(1,3)*norm(uz)*sin(phi0))/(cos(phi0)*norm(uz));(uz(2)-L(2,3)*norm(uz)*sin(phi0))/(cos(phi0)*norm(uz));(uz(3)-L(3,3)*norm(uz)*sin(phi0))/(cos(phi0)*norm(uz))];
% L(:,1)=cross(L(:,2),L(:,3)); 

L = zeros(3,3);
L(:,3) = fz / g;
% L(:,3) = [0; 0; 1];
sinTeta = L(2,3);
teta1 = asin(sinTeta)
cosGamma = L(3,3) / cos(teta1);
gamma1 = -abs(acos(cosGamma))
L( :,3) = [-cos(teta1) * sin(gamma1); sinTeta; cos(teta1)*cosGamma];

L(:,2) = [(uz(1)-L(1,3)*norm(uz)*sin(phi0))/(cos(phi0)*norm(uz));(uz(2)-L(2,3)*norm(uz)*sin(phi0))/(cos(phi0)*norm(uz));(uz(3)-L(3,3)*norm(uz)*sin(phi0))/(cos(phi0)*norm(uz))];
cosPsi = L(2,2) / cos(teta1);
psi1 = -acos(cosPsi)

L(:, 2) = [sin(psi1)*cosGamma + cosPsi*sinTeta*sin(gamma1); cosPsi*cos(teta1); sin(psi1)*sin(gamma1) - cosPsi*sinTeta*cosGamma];
L(:, 1) = [cosPsi*cosGamma - sin(psi1)*sinTeta*sin(gamma1); -sin(psi1)*cos(teta1); cosPsi*sin(gamma1) + sin(psi1)*sinTeta*cosGamma]; 


% ????? ?????? ?????? ? ?????? ?????? ? ???????
 
% ?????????????? ???? ?????�??? ?????????

Az=L; % ??�. ??????? Az(0) = L(0), Ax(0) = E
 
Ax=eye(3);

 % ????????? ??�?????? ??????? (? ??�?????? ?????? ??????? ????? ?? ?????)
for i = 1:Ni
   lambda(i) = ld0;
   phi(i) = phi0;
   hs(i) = h0;
   Ves(i) = Ve0;
   Vns(i) = Vn0;
   Vhs(i) = Vh0;
   psi(i) = psi1;
   gam(i) = gamma1;
   tet(i) = teta1;
end


for k = Ni:(size(t)-1)
    
%1  ???
delta = t(k+1) - t(k);

%2
% ????? ?????
omega = sqrt(omega1(k)^2 + omega2(k)^2 + omega3(k)^2);
% ???????????? ??????? (3.12)
c1 = sin(omega*delta)/omega;
c2 = (1-cos(omega*delta))/(omega^2) ;


Omega = [0 , omega3(k), - omega2(k);...  % ??????? (3.4) ??? Az
     -omega3(k),  0 , omega1(k);...
     omega2(k), - omega1(k) , 0];
 
Az = (eye(3) + c1*Omega+ c2* Omega^2)*Az; % A_i+1  ???????????? ??????? (3.12)

%3

Re= a/sqrt(1-e*(sin(phi(k)))^2);% + hs(k);  (3.7)
Rn= a*(1-e)/(1-e*(sin(phi(k)))^2)^(3/2);% + hs(k); (3.7)

%  g= 9.78030*(1+0.0053020*(sin(phi(k)))^2-0.0000070*(sin(2*phi(k)))^2 - 2*hs(k)/a) - 0.00014;
 g = 9.78030*(1+0.0053020*(sin(phi(k)))^2-0.0000070*(sin(2*phi(k)))^2) - 0.00014 - 0.000003086*hs(k);
 %g = 9.78030*(1 - 2*hs(k)/a + (3/4)*e*(sin(phi(k)))^2);
%  g = 9.8155;

% ?????????? Ux
uE = 0;  
uN = u*cos(phi(k));
uH = u*sin(phi(k));

 % ??????? (3.6). Omega - ?????? ??????? ???????? ????????�??????
 % ???????????? ???????????? ?????
omegaE = -Vns(k)/(Rn+hs(k));  % Omeg1
omegaN = Ves(k)/(Re+hs(k));   % Omeg2
omegaH = omegaN*tan(phi(k));  %Omeg3

% ?????�????? ??????? ????? (�???? ???�????? Ax)
Omega = [0 , omegaH + uH, - omegaN-uN;...
     -omegaH-uH,  0 , omegaE+uE;...
     omegaN+uN, - omegaE-uE , 0];

 % ???????�?? ????, �?? ????
 omega = sqrt((omegaE+uE)^2 + (omegaN+uN)^2 + (omegaH + uH)^2);
 
c1 = sin(omega*delta)/omega;
c2 = (1-cos(omega*delta))/(omega^2) ;
F = eye(3) + c1*Omega + c2* Omega^2;
Ax = F*Ax; % A_i+1

%4
L = Az*Ax'; 

% detL = det(L)% ?????????? ?????????? ???????????? ???????????? ????????�?????? ???????????? (????? ??????? 3.7)
fz = [z1(k); z2(k); z3(k)];
 % f (?????????/????) ? ??????? ????????? (z1, z2, z3)
% ??????? ? ??????? Ox (???)
fy = L'*fz;  % ??????? ? ???????�????? ??????????; fy - ????/????????? ???????????? ?????

% ????? ?????? ? ?????? ???????

%5
% (3.14 (2))
dotVe = (omegaH + 2*uH)*Vns(k) - (omegaN + 2*uN)*Vhs(k) + fy(1,1);
dotVn = -(omegaH + 2*uH)*Ves(k) + (omegaE + 2*uE)*Vhs(k) + fy(2,1);
dotVh= (omegaN + 2*uN)*Ves(k) - (omegaE + 2*uE)*Vns(k) + fy(3,1) - g;

% fy
            
for i7 = 1 : size(Ve)
         if t(k) == t2(i7)
            teta2 = pitch(i7);
            gamma2 = roll(i7);
            psi2 = head(i7);
            L1( :,3) = [-cos(teta2) * sin(gamma2); sin(teta2); cos(teta2)*cos(gamma2)];
            L1(:, 2) = [sin(psi2)*cos(gamma2) + cos(psi2)*sin(teta2)*sin(gamma2); cos(psi2)*cos(teta2); sin(psi2)*sin(gamma2) - cos(psi2)*sin(teta2)*cos(gamma2)];
            L1(:, 1) = [cos(psi2)*cos(gamma2) - sin(psi2)*sin(teta2)*sin(gamma2); -sin(psi2)*cos(teta2); cos(psi2)*sin(gamma2) + sin(psi2)*sin(teta2)*cos(gamma2)]; 
            L1 = L1';
            fy1 = L1'*fz;
            dotVe1 = (omegaH + 2*uH)*Vn(i7) - (omegaN + 2*uN)*Vh(i7) + fy1(1,1);
            dotVn1 = -(omegaH + 2*uH)*Ve(i7) + (omegaE + 2*uE)*Vh(i7) + fy1(2,1);
            dotVh1 = (omegaN + 2*uN)*Ve(i7) - (omegaE + 2*uE)*Vn(i7) + fy1(3,1) - g;
            norm(fy - fy1)
            t2(i7)
            deltaDotV1(i7) = (dotVe - dotVe1);
            deltaDotV2(i7) = (dotVn - dotVn1);
            deltaDotV3(i7) = (dotVh - dotVh1);
        
         end
end


%6
% ????????? ??? ????????. ????? ??????
Ves(k+1)=  Ves(k) + delta*dotVe;  % V1
Vns(k+1)=  Vns(k) + delta*dotVn;  % V2
Vhs(k+1)=  Vhs(k) + delta*dotVh;

%7
% ??????? (3.15). ????? ??????. ?????????? ???????? ? ????????�?????
% % ???????
lambda(k+1) = lambda(k)+delta*Ves(k)/((Re+hs(k))*cos(phi(k)));
phi(k+1)= phi(k)+delta*Vns(k)/(Rn+hs(k));
hs(k+1) = hs(k) + delta*Vhs(k);

end

dotVe1 = (omegaH + 2*uH)*Vn(end) - (omegaN + 2*uN)*Vh(end) + fy(1,1);
dotVn1 = -(omegaH + 2*uH)*Ve(end) + (omegaE + 2*uE)*Vh(end) + fy(2,1);
dotVh1 = (omegaN + 2*uN)*Ve(end) - (omegaE + 2*uE)*Vn(end) + fy(3,1) - g;
deltaDotV1(end) = (dotVe - dotVe1);
deltaDotV2(end) = (dotVn - dotVn1);
deltaDotV3(end) = (dotVh - dotVh1);

sizeFrac = size(phi) / size(long);

x_0 = (R + h(1)) * long(1);
y_0 = (R + h(1)) * log(tan(pi/4 + 0.5*lat(1)));
for i1 = 1 : size(long) 
    x(i1) = (R + h(i1)) * long(i1) - x_0;
    y(i1) = (R + h(i1)) * log(tan(pi/4 + 0.5*lat(i1))) - y_0;
end

x_1_0 = (R + hs(1)) * lambda(1);
y_1_0 = (R + hs(1)) * log(tan(pi/4 + 0.5*phi(1)));

x1 = zeros(901,1);
y1 = zeros(901,1);
h1 = zeros(901,1);
for i2 = 1 : size(lambda')
    
%     if mod(i2,99) == 0
%         i3 = i2 / 99;
        i3 = i2;
        x1(i3) = (R + hs(i2)) * lambda(i2) - x_1_0;
        y1(i3) = (R + hs(i2)) * log(tan(pi/4 + 0.5*phi(i2))) - y_1_0;
        h1(i3) = hs(i2);
%     end
    
end    

for i2 = 1 : size(lambda)
    long(i2) = (R + h(i2)) * sin(phi(i2)) * cos(lambda(i2));
    lat(i2) = (R + h(i2)) * sin(phi(i2)) * sin(lambda(i2));
end



% ??????
figure(1);clf;
plot3(x,y,h,'k','LineWidth' ,3);
hold on;
plot3(x1,y1,h1,'g','LineWidth' ,3);


r1 = [(R + hs(k)) * sin(phi(k)) * cos(lambda(k)),
        (R + hs(k)) * sin(phi(k)) * sin(lambda(k)),
        (R + hs(k)) * cos(phi(k))];
    
r2 = [(R + h(901)) * sin(lat(901)) * cos(long(901)),
        (R + h(901)) * sin(lat(901)) * sin(long(901)),
        (R + h(901)) * cos(lat(901))];

r = norm(r1 - r2)


deltaV1 = zeros(901, 1);
deltaV2 = zeros(901, 1);
deltaV3 = zeros(901, 1);

for i4 = 1 : size(Ve)
    
%     index = 1;
    
    for i7 = 1 : size(Ves')
         if t(i7) == t2(i4)
             index = i7;
             deltaV1(i4) = (Ve(i4) - Ves(index));
    deltaV2(i4) = (Vh(i4) - Vhs(index));
    deltaV3(i4) = (Vn(i4) - Vns(index));
         end
    end
    
    
end


deltaV1(end) = (Ve(end) - Ves(end));
deltaV2(end) = (Vh(end) - Vhs(end));
deltaV3(end) = (Vn(end) - Vns(end));

figure(8);clf;
plot(t2(3:901),deltaDotV3);
hold on;
grid on;

figure(9);clf;
plot(t2(3:901),deltaDotV2);
hold on;
grid on;

figure(10);clf;
plot(t2(3:901),deltaDotV1);
hold on;
grid on;

figure(7);clf;
plot(t2,deltaV1);
hold on;
grid on;

figure(6);clf;
plot(t2,deltaV2);
hold on;
grid on;

figure(5);clf;
plot(t2,deltaV3);
hold on;
grid on;

figure(2);clf;
plot(t2,Ve)
hold on
plot(t,Ves)
hold on

figure(3);clf;
plot(t2,Vh)
hold on
plot(t,Vhs)
hold on

figure(4);clf;
plot(t2,Vn)
hold on
plot(t,Vns)
hold on


grid on;
  
   