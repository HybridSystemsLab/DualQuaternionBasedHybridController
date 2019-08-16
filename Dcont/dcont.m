function [xdot] = dcont(t,x)

global tnoise NOISEr NOISEx NOISEy NOISEz i s etar

xsys = x(1:9);
tau = x(10);
w = x(11:13);
v = x(14:16);
m = xsys(9);
%------------Noise----------%
    tn = tau;
    tmp = abs(tnoise-tn);
    [row col] = min(tmp);
    pnoiser = NOISEr(col);  % process noise r
    pnoisex = NOISEx(col);  % process noise x
    pnoisey = NOISEy(col);  % process noise y
    pnoisez = NOISEz(col);  % process noise x
%---------8x8 inetia matrix----------%
% J = [4.97 0.0 0.0;
%      0.0 6.16 0.0;
%      0.0 0.0  8.37];
J = [1 0.1 0.15;
     0.1 0.63 0.05;
     0.15 0.05  0.85];
%-------states--------%
e = [pnoiser pnoisex pnoisey pnoisez]/norm([pnoiser pnoisex pnoisey pnoisez]);
m = 0.1*randi([0 2],1,1);
mv = 0.0*randi([0 2],1,1);
qr = (xsys(1:4)+m*e')/norm(xsys(1:4)+m*e');
eta1 = qr(1);
mu1 = qr(2:4);
q1s = [xsys(1); -xsys(2:4)];
q2 = [xsys(5); xsys(6:8)];
qm = 2*quatmultiply(q1s',q2');
pb = qm(2:4)'+5*[pnoisex;pnoisey;pnoisez];
qt = 0.5*quatmultiply(qr',[0 pb']);
eta2 = qt(1);%xsys(5);
mu2 = qt(2:4)';
%mu2 = xsys(6:8)+1*[pnoisex;pnoisey;pnoisez];
w = w+mv*[pnoisex;pnoisey;pnoisez];
v = v+mv*[pnoisex;pnoisey;pnoisez];
%  sgn=0;
% s(1)=sgn; 
if eta1>=0
sgn   = 1;
elseif eta1<0
        sgn = -1;
end
   s(i) = sgn;
   etar(i) = eta1;  
     i = i+1;
   
%while i<=1000000

%end
   
%-------Controllerstates--------%
%e = [pnoiser pnoisex pnoisey pnoisez]/norm([pnoiser pnoisex pnoisey pnoisez]);
%m = 0.1*randi([0 2],1,1);
qrn = (xsys(1:4)+m*e')/norm(xsys(1:4)+m*e');
etac = qrn(1);
mu1c = qrn(2:4);
eta2c = eta2;%xsys(5);
mu2c = mu2;%xsys(6:8)+1*[pnoisex;pnoisey;pnoisez];
wc = w+mv*[pnoisex;pnoisey;pnoisez];
vc = v+mv*[pnoisex;pnoisey;pnoisez];

% ----trans-rot vel term----%
V = v;%+cross(w,pb);

%-----------GAINS--------%
c = 0.5;
kdw = 0.5*eye(3);

% --------Feedback-------------% 
%This does not work since the angular vel swap for rigid body dynamicsd
T = -c*sgn*mu1c- kdw*wc; 
%------------------------------------% 
% Kinematics
etad  = -0.5*mu1'*wc;
eta2d = -0.5*(mu2'*wc+mu1'*vc);
mud1   = 0.5*(eta1*wc +cross(mu1,wc));
mu2d  = 0.5*(eta2*wc+eta1*vc+cross(mu1,vc)+cross(mu2,wc));
hd = 0;

%-------angular vel part of combined Vel-------%
wd = J\(T+ cross(J*w,w)); 
%---------Linear dynamics--------$
m = 1;
B = [1/m  0 0;0  1/m 0;0  0 1/m];
kdv = 0.5;
F = -(etac*mu2c-eta2c*mu1c-cross(mu1c,mu2c))-1*kdv*vc;
%F = -1*kdw*v;

vd = -cross(w,v)+1*F;

% if t == 0;
%     i=1;
% else 
%     i=i+1;
% end
%s(i)= sgn;

xdot = [etad;mud1;eta2d;mu2d;hd;1;wd;vd];





