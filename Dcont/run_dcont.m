global delta tnoise NOISEr NOISEx NOISEy NOISEz s i etar pnoiset
s = nan(100000,1);
s(1,1) = 1;
i = 2;
etar = nan(100000,1);
pnoiset = nan(100000,1);

%-----------------NOISE-----------%
N=5500;
Fs = 100;
tnoise = (0:N-1)/Fs;
sigma = 1;%*sqrt(0.04);
NOISEr = sigma*randn(size(tnoise));
NOISEx = sigma*randn(size(tnoise));
NOISEy = sigma*randn(size(tnoise));
NOISEz = sigma*randn(size(tnoise));

% % %-----------IC in the paper---------%
% % %-----Mayhew Journal PAPER-------%
vl = [1 2 3]/norm([1 2 3]);
q1 = 0;
q2 = 0.4243;%vl(1);
q3 = 0.5657;%vl(2);
q4 = 0.7071;%vl(3);
% % Translational initial conditions
% % 
p0 = 0;
p1 = 25;
p2 = 25;
p3 = 25;
% % 
% % combined angular and linear vel
w1 = 0.2;
w2 = 0.4;
w3 = 0.6;

v1 = 0.1;
v2 = 0.2;
v3 = 0.3;


% % %-----------IC in the paper---------%
% % %-----SIM-2 IN GNC PAPER Figure 3-------%
% % % -----Fillipe's paper-----%
% 
% q1 = -0.4618;
% q2 = 0.1917;
% q3 = 0.7999;
% q4 = 0.3320;
% % 
% % -----Fillipe's paper-----%
% p0 = 0;
% p1 = 2;
% p2 = 2;
% p3 = 1;
% 
% % % ------ combined angular and linear vel Fillipe's paper-------%
% w1 = -0.1;
% w2 = 0.2;
% w3 = -0.3;
% 
% v1 = 0.1;
% v2 = -0.2;
% v3 = 0.3;
% %----------------------------------------------------------%

%Rotation followed by translation
qr = [q1 q2 q3 q4];
pri = [p0 p1 p2 p3];
qts = 0.5*quatmultiply(qr,pri);

qt = qts; %[q5 q6 q7 q8];
hi = -1;
taui = 0;


ws = 1*[w1 w2 w3 v1 v2 v3];

xi = [qr qt hi taui ws];
etar(1,1) = q1;


% simulation horizon
T = [0 20];                                                                


options = odeset('RelTol',1e-3,'MaxStep',1/1000);

% simulate working old in the paper
[t, x] = ode45(@dcont,T,xi',options);
