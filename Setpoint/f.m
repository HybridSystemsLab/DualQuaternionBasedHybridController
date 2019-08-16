function xdot = f(x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by BMalladi)
%
% Project: Simulation of a hybrid system (Rigid body attitude)
%
% Name: f.m
%
% Description: Flow map
%
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global tnoise NOISEr NOISEx NOISEy NOISEz 
%global s k %dcont switching

xsys = x(1:9);
tau = xsys(9);
%------------Noise----------%
    tn = tau;
    tmp = abs(tnoise-tn);
    [row col] = min(tmp);
    pnoiser = NOISEr(col);  % process noise r
    pnoisex = NOISEx(col);  % process noise x
    pnoisey = NOISEy(col);  % process noise y
    pnoisez = NOISEz(col);  % process noise x 
    
%---------J = diag(10v) intertial tensor----------%
J = [10 0 0;
     0 20 0;
     0 0  30];
%------------measurement error-------%
e = [pnoiser pnoisex pnoisey pnoisez]/norm([pnoiser pnoisex pnoisey pnoisez]);
m = 0.1*randi([0 2],1,1);
qr = xsys(1:4)+m*e';
nrmdq = norm(qr);

%-------states--------%
qrn = qr/nrmdq; % Normalized to comply with unit norm constraint
eta = qrn(1);
mu = qrn(2:4);
w = xsys(5:7);
h   = xsys(8);
% % %-----------Dcont controller----------%
% % if eta>=0
% % h   = 1;
% % elseif eta<0
% %         h = -1;
% % end
% %    s(k) = h; 
% %     k = k+1;

%-------Controllerstates--------%
etac = qrn(1);
muc = qrn(2:4);
wc = w;%+[0.0*pnoisex;0.0*pnoisey;0.0*pnoisez];
%-----------GAINS--------%
% c = 5;
% kdw = 1*eye(3);

c = 1; % Spring 
kdw = 4*eye(3);  % Damping

% --------Feedback-------------% 
T = -c*h*muc- kdw*wc; 


%---------------Kinematics-------------%  
etad  = -0.5*mu'*w;
mud   = 0.5*(eta*w +cross(mu,w));


%-------------Kinetics-------%
wd = J\(T+ cross(J*w,w)); 

%---------Logic variable--------%
hd = 0;

%--------Timer for noise-------%
taud = 1;
%----------------------------------------------------%

xdot = [etad;mud;wd;hd;taud];

