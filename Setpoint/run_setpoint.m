%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file       Project: HyEQ Toolbox  @ Hybrid Dynamics and Control
% Lab, https://hybrid.soe.ucsc.edu/lab/index.php?n=Main.Software
%
% Filename: run_switch.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all

format long
global delta tnoise NOISEr NOISEx NOISEz NOISEy 
%global s k %dcont switching

delta =0.4;  % >1 cont controller

%-----------Dcont controller----------$
s = nan(100000,1);
s(1,1) = 1;
k = 2;

%-----------------NOISE-----------%
N=5500;
Fs = 100;
tnoise = (0:N-1)/Fs;
sigma = 1;
NOISEr = sigma*randn(size(tnoise));
NOISEx = sigma*randn(size(tnoise));
NOISEy = sigma*randn(size(tnoise));
NOISEz = sigma*randn(size(tnoise));


% % -----Mayhew's TAC Journal paper-----%
va = [1 2 3];
v = va/norm(va);
%----------Figure 3 IC-------------------%
%%%Close to 0 and results in chattering for Dcont controller
% q1 = 0;
% q2 = v(1);
% q3 = v(2);
% q4 = v(3);
% 
% w1 = 0;
% w2 = 0;
% w3 = 0;

%----------Figure 4 IC-------------------%
mu0 = sqrt(1-0.2^2)*v;
q1 = -0.2;
q2 = mu0(1);
q3 = mu0(2);
q4 = mu0(3);

w1 = 0.5*v(1);
w2 = 0.5*v(2);
w3 = 0.5*v(3);


% Check the norm of q

nq = norm([q1,q2,q3,q4]);


%Quaternion initial condition
qi = [q1 q2 q3 q4];

% Angular velocity initial condition
wi = [w1 w2 w3];

% logic variable initial condition
hi = 1; 

% Timer initial condition for noise
taui = 0;


xi = [qi wi hi taui];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

% simulation horizon
T = [0 70];                                                                
J = [0 10];


%------Dcont controller------%
%J = [0 1];
%rule = 2;

options = odeset('RelTol',1e-6,'MaxStep',1/1000);

% simulate working old in the paper
[t j x] = HyEQsolver( @f,@g,@C,@D,xi',T,J,rule,options);

% % New files for updated A2 region
% [t j x] = HyEQsolver( @f4,@g4,@C4,@D4,x0',T,J,rule,options);
  

 %%
%plot hybrid arc   
figure(1)
subplot(2,1,1)
hold on
plot(t,x(:,1),'Linewidth',2); 
hold on
plot(t,x(:,2));
plot(t,x(:,3));
plot(t,x(:,4));
grid on
xlabel('time (sec)')                    
ylabel('$q_r$')  
legend('\eta_r','\mu_{r_1}','\mu_{r_2}','\mu_{r_3}')

subplot(2,1,2)
hold on
 plot(t,x(:,8).*x(:,1))
%----plot discont controller -------% 
%plot(t,s(1:length(x),:).*x(:,1))
grid on
hold on
xlabel('time (sec)')
ylabel('$h\eta$')    

%% Angular Velocity
figure(2)
subplot(3,1,1)
hold on
plot(t,x(:,5))
xlabel('time (sec)')
ylabel('$\omega_x$')
grid on
subplot(3,1,2)
hold on
plot(t,x(:,6))
xlabel('time (sec)') 
ylabel('$\omega_y$')
grid on
subplot(3,1,3)
hold on
plot(t,x(:,7))
xlabel('time (sec)') 
ylabel('$\omega_z$')
grid on   

%% Norm of angular velocity
for i = 1:length(t)
    wt(i) = (x(i,5:7)*x(i,5:7)');
    i = i+1000;
end

%% Control errort

for i = 1:length(t)
    %-------Controllerstates--------%
    muc = x(i,2:4);
    wc = x(i,5:7);
    h  = x(i,8);
    %-----------GAINS--------%
    c = 1;
    kdw = 4;
     %-------DISCONT CONTROLLER-------%
%     h = s(i);
    %-----Torque caluclations-----%
    tau = -c*h*muc- kdw*wc;
    tauT(i) = tau*tau';
    clear muc wc tau
end

%Area under the curve
Zt = sqrt(cumtrapz(t,tauT));

%%
figure(3)
subplot(2,1,1)
plot(t,wt)
grid on
xlabel('time (sec)')
ylabel('$\|\omega\|$') 
hold on
subplot(2,1,2)
plot(t,Zt)
grid on
hold on
xlabel('time (sec)')
ylabel('$\sqrt{\int_0^t\tau^\top \tau dt}$') 

%% Principle angle
 for i = 1:length(t)
     th = 2*acosd(x(i,1));
 if th >= 180
     thn(i) = th -360;
 else
     thn(i) = th;
 end
 i = i+100;
 end
%%
figure(4)
plot(t,thn)
grid on
hold on
plot(t,180*ones(1,length(t)),'r','linewidth',2)
plot(t,-180*ones(1,length(t)),'r','linewidth',2)
xlabel('time (sec)')
ylabel('Principal angle (deg)') 
