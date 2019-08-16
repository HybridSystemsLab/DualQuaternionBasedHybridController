function xdot = ftk(x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by Giampiero Campa)
%
% Project: Simulation of a hybrid system (Bouncing Ball)
%
% Name: f.m
%
% Description: Flow map
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global tnoise NOISEr NOISEx NOISEy NOISEz

%------------------Desired frame----------------%
xsysd = x(1:8);
wd = x(9:11);
vd = x(12:14);
%----------desired orientaion and position-----%
qdr = xsysd(1:4);
qdt = xsysd(5:8);
qdrs = [xsysd(1); -xsysd(2:4)];
qm = 2*qnpdt(qdrs,qdt);
pd = qm(2:4);

%---------Desired frame Pose--------%
md = 1000;
qdrDot = 0.5*quatmultiply(qdr',[0 wd']);
wdDot = zeros(3,1);
qdtDot = 0.5*quatmultiply(qdr',[0 vd'])+ 0.5*quatmultiply(qdt',[0 wd']);
%0.5*qnpdt([0;pdDot],qdr)+ 0.5*qnpdt([0;pd],qdrDot);
%vdDot =  zeros(3,1);
nu0 = -0.9; %rad/sec
vdDot = [0;0;-9.80665]/md-cross(wd,vd);%[0 -nu0 0;2*nu0 0 0;0 0 0]*vd;

%-------------------Body frame----------------%

xsysb = x(15:22);
wb = x(23:25);
vb = x(26:28);
h   = x(29);
tau = x(30);
qbr = xsysb(1:4);
qbt = xsysb(5:8);
%-------cross check if the postion values or correct--------%
qbrs = [qbr(1); -qbr(2:4)];
qmb = 2*quatmultiply(qbrs',qbt');
pb = qmb(2:4);
%------------Noise----------%
    tn = tau;
    tmp = abs(tnoise-tn);
    [row col] = min(tmp);
    pnoiser = NOISEr(col);  % process noise r
    pnoisex = NOISEx(col);  % process noise x
    pnoisey = NOISEy(col);  % process noise y
    pnoisez = NOISEz(col);  % process noise x
%---------8x8 inetia matrix----------%
%J = diag(10*([1 2 3]/norm([1 2 3])));
% J = 2*[1 0 0;
%      0 1 0;
%      0 0  1];
J = [1 0.1 0.15;
    0.1 0.63 0.05;
    0.15 0.05  0.85];
%------------measurement error-------%
e = [pnoiser pnoisex pnoisey pnoisez]/norm([pnoiser pnoisex pnoisey pnoisez]);
%e = pnoiser;%/norm(pnoiser);
m = 0.01*randi([0 2],1,1);
qbrc = qbr+m*e';

nrmdq = norm(qbrc);%qr(1)*qt(1)+dot(qr(2:4),qt(2:4));

%-------states--------%
qbrn = qbrc/nrmdq;
qbtn = qbt/nrmdq;

barqrc = quatmultiply(quatinv(qdr'),qbrn');
murc = barqrc(2:4);
etarc = barqrc(1);
barqr = quatmultiply(quatinv(qdr'),qbrn');
barqrinv = quatinv(barqr);

%barqt = qnpdt(quatinv(qdt')',qbrn)+qnpdt(quatinv(qdr')',qbtn);

pdb = quatmultiply(quatmultiply(barqrinv,[0 pd']),barqr);
barqt =  0.5*quatmultiply(barqr,[0 pb]-pdb);%0.5*qnpdt(barqr,[0;pb]-pdb);
barqtinv =  [barqt(1) -barqt(2) -barqt(3) -barqt(4)];
barqtc =  0.5*quatmultiply(barqrc,[0 pb]-pdb);
etatc = barqtc(1);
mutc =  barqtc(2:4);
%-----------GAINS--------%
% c = 5;
% kdw = 1*eye(3);

kp = 4;
kdw = 4*eye(3);

wdDotbc = quatmultiply(quatmultiply(barqrinv,[0 wdDot']),barqr);
wdbc = quatmultiply(quatmultiply(barqrinv,[0 wd']),barqr);
% vdDotb = qnpdt(qnpdt(barqrinv,[0;vdDot]),barqr);
% vdb = qnpdt(qnpdt(barqrinv,[0;vd]),barqr);
vdDotbc = quatmultiply(quatmultiply(barqrinv,[0 vdDot']),barqr)...
          +quatmultiply(quatmultiply(barqtinv,[0 wdDot']),barqr)...
          +quatmultiply(quatmultiply(barqrinv,[0 wdDot']),barqt);
vdbc = quatmultiply(quatmultiply(barqrinv,[0 vd']),barqr)...
          +quatmultiply(quatmultiply(barqtinv,[0 wd']),barqr)...
          +quatmultiply(quatmultiply(barqrinv,[0 wd']),barqt);     


% --------Feedback-------------% 
tau = -kp*h*murc'- kdw*wb-cross(J*wb,wb)+kdw*wdbc(2:4)'+J*wdDotbc(2:4)'-J*cross(wb,wdbc(2:4))';

%------------------------------------% 
% % % Kinematics
%  etad  = -0.5*mu1'*wc;
% % eta2d = -0.5*(mu2'*wc+mu1'*vc);
%  mud1   = 0.5*(eta1*wc +cross(mu1,wc));
% % mu2d  = 0.5*(eta2*wc+eta1*vc+cross(mu1,vc)+cross(mu2,wc));
hd = 0;
%-------angular vel part of combined Vel-------%
%wd = J\(T+ cross(J*wb,wb)); 
%---------Linear dynamics--------$
mb = 1;
%B = [1/m  0 0;0  1/m 0;0  0 1/m];

kdv = 4;
F =mb*(-1*kp*(etarc*mutc'+etatc*murc'-cross(murc',mutc'))-1*kdv*(vb-vdbc(2:4)')+cross(wb,vb)+vdDotbc(2:4)'- cross(wb,vdbc(2:4)')-cross(vb,wdbc(2:4)'));


%vd = -cross(w,v)+1*F;

%-------Rigid body dynamics---------%
qbrDot = 0.5*quatmultiply(qbrn',[0 wb']);
wbDot =  J\(tau+cross(J*wb,wb));
qbtDot = 0.5*quatmultiply(qbrn',[0 vb'])+ 0.5*quatmultiply(qbtn',[0 wb']);
vbDot = -cross(wb,vb)+F/mb;
%----------------------------------------------------%

xdot = [qdrDot';qdtDot';wdDot;vdDot;qbrDot';qbtDot';wbDot;vbDot;hd;1];

