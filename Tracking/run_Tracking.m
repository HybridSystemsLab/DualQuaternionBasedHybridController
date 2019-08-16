%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file       Project: HyEQ Toolbox  @ Hybrid Dynamics and Control
% Lab, http://www.u.arizona.edu/~sricardo/index.php?n=Main.Software
%
% Filename: run_switch.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all

%format long
global delta tnoise NOISEr NOISEx NOISEz NOISEy

delta =0.1;


%-----------------NOISE-----------%
N=5500;
Fs = 100;
tnoise = (0:N-1)/Fs;
sigma = 1;
NOISEr = sqrt(sigma)*randn(size(tnoise));
NOISEx = sigma*randn(size(tnoise));
NOISEy = sigma*randn(size(tnoise));
NOISEz = sigma*randn(size(tnoise));


%------------Desired frame IC-----------%
% % -----Desired orientation-----%
qd1 = 1;
qd2 = 0;
qd3 = 0;
qd4 = 0;

%-----Desired position-----%
pd0 = 0;
pd1 = 0;
pd2 = 0;
pd3 = 0;


%------Desired  combined angular and linear vel-------%
wd1 = -0.1;
wd2 = 0.65;
wd3 = -0.2;

vd1 = -0.5;
vd2 = .1;
vd3 = 0.1;

%Rotation followed by translation
qrdes = [qd1 qd2 qd3 qd4];
prdes = [pd0 pd1 pd2 pd3];
qtdes = 0.5*quatmultiply(qrdes,prdes);

whatdes = [wd1 wd2 wd3 vd1 vd2 vd3];

%--------------Body frame IC-----------%
% % -----Body orientation-----%
va = [1 2 3];
v = va/norm(va);
qb1 = 0.1;
mu0 = sqrt(1-qb1^2)*v;
qb2 = mu0(1);
qb3 = mu0(2);
qb4 = mu0(3);
% qb1 = 0;
% qb2 = 0.4243;
% qb3 = 0.5657;
% qb4 = 0.7071;
qrbody = [qb1 qb2 qb3 qb4];

nq = norm(qrbody)

barqr0 = quatmultiply(quatinv(qrdes),qrbody);
pdb0 = quatmultiply(quatmultiply(quatinv(barqr0),prdes),barqr0);
%wdb0 = qnpdt(qnpdt(quatinv(barqr0')',[0 wd1 wd2 wd3]'),barqr0);
vdb0 = quatmultiply(quatmultiply(quatinv(barqr0),[0 vd1 vd2 vd3]),barqr0);

%-----Body position-----%
pb0 = pdb0(1,1);
pb1 = pdb0(1,2)+2;
pb2 = pdb0(1,3)+2;
pb3 = pdb0(1,4)+1;

%------Body  combined angular and linear vel-------%
% wb1 = 0;
% wb2 = 0;
% wb3 = 0;
wb1 = -0.6*v(2);
wb2 = 0.6*v(2);
wb3 = 0.1;

vb1 = 1;
vb2 = .5;
vb3 = 0.5;

%Rotation followed by translation
prbody = [pb0 pb1 pb2 pb3];
qtbody = 0.5*quatmultiply(qrbody,prbody);

whatbody = 1*[wb1 wb2 wb3 vb1 vb2 vb3];

taui = 0;

hi = -1;


xi = [qrdes qtdes whatdes qrbody qtbody whatbody hi taui];



% simulation horizon
T = [0 20];                                                                
J = [0 10];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-3,'MaxStep',1);

% simulate working old in the paper
[t j x] = HyEQsolver( @ftk,@gtk,@Ctk,@Dtk,xi',T,J,rule,options);

  

%%
j = 1;
for i = 1:10:length(t)
qdr = x(i,1:4)';
qdrs = [qdr(1); -qdr(2:4)];
qdt = x(i,5:8)';
qdm = (2*quatmultiply(qdrs',qdt'))';
pd = qdm(2:4)';
wd = x(i,9:11)';
vd = x(i,12:14)';
qbr = x(i,15:18)';
qbrs = [qbr(1); -qbr(2:4)];
qbt = x(i,19:22)';
qbm = (2*quatmultiply(qbrs',qbt'))';
pb = qbm(2:4)';
barqr = quatmultiply(quatinv(qdr'),qbr');
barqrinv = quatinv(barqr);
pdb(j,1:4) = quatmultiply(quatmultiply(barqrinv,qdm'),barqr);
barqt =  0.5*quatmultiply(barqr,[0 pb]-pdb(j,1:4));
barqtinv = [barqt(1) -barqt(2) -barqt(3) -barqt(4)];

wdb(j,1:4) = quatmultiply(quatmultiply(barqrinv,[0 wd']),barqr);
vdb(j,1:4) = quatmultiply(quatmultiply(barqrinv,[0 vd']),barqr);...
            +quatmultiply(quatmultiply(barqtinv,[0 wd']),barqr)...
            +quatmultiply(quatmultiply(barqrinv,[0 wd']),barqt);

pbb(j,1:4) = qbm;
pdd(j,1:4) = qdm;
pbd(j,1:4) = quatmultiply(quatmultiply(barqr,qbm'),barqrinv);
ti(j,1) = t(i);
j = j+1;
clear barq barqinv pb
end



%%
figure(1)
plot3(pdb(:,2),pdb(:,3),pdb(:,4))
hold on
plot3(pdb(1,2),pdb(1,3),pdb(1,4),'*')
hold on
plot3(pbb(:,2),pbb(:,3),pbb(:,4))
hold on
plot3(pbb(1,2),pbb(1,3),pbb(1,4),'*')
grid on
xlabel('${p_x}_{\{d,b\}}$')
ylabel('${p_y}_{\{d,b\}}$')
zlabel('${p_z}_{\{d,b\}}$')
legend('Reference trajectory','Reference intial position','Rigid body trajectory', 'Rigid body intial position')

%%
figure(2)
plot3(pdd(:,2),pdd(:,3),pdd(:,4))
hold on
plot3(pdd(1,2),pdd(1,3),pdd(1,4),'*')
hold on
plot3(pbd(:,2),pbd(:,3),pbd(:,4))
hold on
plot3(pbd(1,2),pbd(1,3),pbd(1,4),'*')
grid on
xlabel('${p_x}^d_{\{d,b\}}$')
ylabel('${p_y}^d_{\{d,b\}}$')
zlabel('${p_z}^d_{\{d,b\}}$')
legend('Reference trajectory','Reference initial position','Rigid body trajectory', 'Rigid body initial position')

 %%
%plot hybrid arc   
figure(3)
subplot(2,1,1)
hold on
plot(t,x(:,1)); 
hold on
plot(t,x(:,29).*x(:,15));
grid on                    
ylabel('$\eta_{\{d,b\}}$')  
legend('$\eta_{d_r}$','$h\eta_{b_r}$')

subplot(2,1,2)
hold on
 plot(t,x(:,29))
%plot(t,s(1:length(x),:).*x(:,1))
grid on
hold on
xlabel('time (sec)')
ylabel('$h$')  

%% Angular Velocites

figure(4)
subplot(3,1,1)
hold on
plot(ti,wdb(:,2)); 
hold on
plot(t,x(:,23));
grid on                   
ylabel('${\omega_x}_{\{d,b\}}$')  
legend('$\omega_{d_x}$','$\omega_{b_x}$')

subplot(3,1,2)
hold on
plot(ti,wdb(:,3)); 
hold on
plot(t,x(:,24));
grid on
hold on
ylabel('${\omega_y}_{\{d,b\}}$')  
legend('$\omega_{d_y}$','$\omega_{b_y}$')

subplot(3,1,3)
hold on
plot(ti,wdb(:,4)); 
hold on
plot(t,x(:,25));
grid on
hold on
xlabel('time (sec)')
ylabel('${\omega_z}_{\{d,b\}}$')  
legend('$\omega_{d_z}$','$\omega_{b_z}$')


%% Linear Velocites

figure(5)
subplot(3,1,1)
hold on
plot(ti,vdb(:,2)); 
hold on
plot(t,x(:,26));
grid on                  
ylabel('${v_x}_{\{d,b\}}$')  
legend('$v_{d_x}$','$v_{b_x}$')

subplot(3,1,2)
hold on
plot(ti,vdb(:,3)); 
hold on
plot(t,x(:,27));
grid on
hold on
ylabel('${v_y}_{\{d,b\}}$')  
legend('$v_{d_y}$','$v_{b_y}$')

subplot(3,1,3)
hold on
plot(ti,vdb(:,4)); 
hold on
plot(t,x(:,28));
grid on
hold on
xlabel('time (sec)')  
ylabel('${v_z}_{\{d,b\}}$')  
legend('$v_{d_z}$','$v_{b_z}$')

%% Position of the desired and body frame expressed in body frame

figure(6)
subplot(3,1,1)
hold on
plot(ti,pdb(:,2)); 
hold on
plot(ti,pbb(:,2));
grid on                  
ylabel('${p_x}_{\{d,b\}}$')  
legend('$p_{d_x}$','$p_{b_x}$')

subplot(3,1,2)
hold on
plot(ti,pdb(:,3)); 
hold on
plot(ti,pbb(:,3));
grid on
hold on
ylabel('${p_y}_{\{d,b\}}$')  
legend('$p_{d_y}$','$p_{b_y}$')

subplot(3,1,3)
hold on
plot(ti,pdb(:,4)); 
hold on
plot(ti,pbb(:,4));
grid on
hold on
xlabel('time (sec)') 
ylabel('${p_z}_{\{d,b\}}$')  
legend('$p_{d_z}$','$p_{b_z}$')


%%
figure(7)
plot3(pdb(:,2),pdb(:,3),pdb(:,4))
hold on
plot3(pdb(1,2),pdb(1,3),pdb(1,4),'*')
grid on
xlabel('$p_{d_x}$')
ylabel('$p_{d_y}$')
zlabel('$p_{d_z}$')
legend('Reference trajectory','Reference initial position')

%%
figure(8)
plot3(pdd(:,2),pdd(:,3),pdd(:,4))
hold on
plot3(pdd(1,2),pdd(1,3),pdd(1,4),'*')
grid on
xlabel('$p^d_{d_x}$')
ylabel('$p^d_{d_y}$')
zlabel('$p^d_{d_z}$')
legend('Reference trajectory','Reference initial position')
