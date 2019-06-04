% initial position
phi0 = 56.8274; % latitude, degrees
lambda0 = 35.7761; % longitude, degrees
h0 = 145; % altitude, meters
psi0 = -105; % heading, degrees

fs = 100; % sampling rate, Hertz
tmax = 1200; % max time, seconds

% FlightGear configuration
fps = 25;
port = 5500;
host = '127.0.0.1';
protocol = 'udp_6dof';

fgpath = 'D:\Program Files\FlightGear 2018.3.1';
fgexe = 'fgfs.exe';
fgcl = ['--timeofday=noon --enable-terrasync --aircraft=org.flightgear.fgaddon.stable_2018.T-50-jsbsim --fdm=external --model-hz=' num2str(fps) ' --in-air --geometry=1024x768 --generic=socket,in,' num2str(fps) ',' host ',' num2str(port) ',udp,' protocol ' --fg-root="' fgpath '\data"'];

% file naming
imuname = 'imu.txt';
trjname = 'trj.txt';
kmlname = 'trj.kml';