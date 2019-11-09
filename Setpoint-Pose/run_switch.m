%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file       Project: HyEQ Toolbox  @ Hybrid Dynamics and Control
% Lab, http://www.u.arizona.edu/~sricardo/index.php?n=Main.Software
%
% Filename: run_switch.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%format long
global delta tnoise NOISEr NOISEx NOISEz NOISEy

delta =0.4;


%-----------------NOISE-----------%
N=550;
Fs = 100;
tnoise = (0:N-1)/Fs;
sigma = 1;%*sqrt(0.04);
NOISEr = sigma*randn(size(tnoise));
NOISEx = sigma*randn(size(tnoise));
NOISEy = sigma*randn(size(tnoise));
NOISEz = sigma*randn(size(tnoise));


% %-----------IC in the paper---------%
% %-----SIM-1 IN GNC PAPER-------%
% q1 = 0;
% q2 = 0.4243;
% q3 = 0.5657;
% q4 = 0.7071;
% % % 
% % % Translational initial conditions
% % 
% p0 = 0;
% p1 = 25;
% p2 = 25;
% p3 = 25;
% 
% % % % combined angular and linear vel
% % 
% w1 = 0.2;
% w2 = 0.4;
% w3 = 0.6;
% 
% v1 = 0.1;
% v2 = 0.2;
% v3 = 0.3;


% % -----Mayhew's Journal paper-----%
vl = [1 2 3]/norm([1 2 3]);
q1 = -0.4618;
q2 = 0.1917;%vl(1);
q3 = 0.7999;%vl(2);
q4 = 0.3320;%vl(3);

%-----Fillipe's paper-----%
p0 = 0;
p1 = 2;
p2 = 2;
p3 = 1;


%------ combined angular and linear vel Fillipe's paper-------%
w1 = -0.1;
w2 = 0.2;
w3 = -0.3;

v1 = 0.1;
v2 = -0.2;
v3 = 0.3;

%Rotation followed by translation
qr = [q1 q2 q3 q4];
pri = [p0 p1 p2 p3];
qts = 0.5*quatmultiply(qr,pri);

qt = qts; %[q5 q6 q7 q8];

taui = 0;

hi = 1;

ws = 1*[w1 w2 w3 v1 v2 v3];

xi = [qr qt hi taui ws];



% simulation horizon
T = [0 25];                                                                
J = [0 10];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-3,'MaxStep',1);

% simulate working old in the paper
[t j x] = HyEQsolver( @f2,@g2,@C2,@D2,xi',T,J,rule,options);

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
 plot(t,x(:,9).*x(:,1))
%plot(t,s(1:length(x),:).*x(:,1))
grid on
hold on
xlabel('time (sec)')
ylabel('$h\eta$')  
%legend('Full state measurement', 'Partial state measurement')
%%
figure(2)
subplot(2,1,1)
plotflows(t,j,x(:,1))
xlabel('time (sec)') 
ylabel('\eta_r')
subplot(2,1,2)
plotjumps(t,j,x(:,1))
xlabel('j') 
ylabel('\eta_r')
grid on                   
          
%%
figure(3)
plot(tnoise,NOISEr)

%%
for i = 1:length(t)
    q1(i,1:4) = [x(i,1) -x(i,2:4)];
    q2(i,1:4) = x(i,5:8);
end

%plot hybrid arc
figure(4)
 pos = 2*quatmultiply(q1,q2);
subplot(2,1,1)
plot(t,pos(:,2))
hold on
plot(t,pos(:,3))
hold on
plot(t,pos(:,4))
xlabel('time (sec)')
ylabel('Rigid Body Position in m')
legend('$r_{p_x}$','$r_{p_y}$','$r_{p_z}$')
grid on
subplot(2,1,2)
plot(t,x(:,14))
hold on
plot(t,x(:,15))
hold on
plot(t,x(:,16))
xlabel('time (sec)') 
ylabel('Rigid Body Velocity in m/sec')
legend('$v_{x}$','$v_{y}$','$v_{z}$')
grid on
%%
figure(5)
subplot(3,1,1)
hold on
plot(t,x(:,11))
xlabel('time (sec)')
ylabel('$\omega_x$')
grid on
subplot(3,1,2)
hold on
plot(t,x(:,12))
xlabel('time (sec)') 
ylabel('$\omega_y$')
grid on
subplot(3,1,3)
hold on
plot(t,x(:,13))
xlabel('time (sec)') 
ylabel('$\omega_z$')
legend('Continuous controller', 'Hybrid Controller')
grid on   
%%
%plot hybrid arc
figure(6)
subplot(3,1,1)
hold on
plot(t,x(:,14),'k')
xlabel('time (sec)')
ylabel('$v_x$')
grid on
subplot(3,1,2)
hold on
plot(t,x(:,15))
xlabel('time (sec)') 
ylabel('$v_y$')
grid on
subplot(3,1,3)
hold on
plot(t,x(:,16))
xlabel('time (sec)') 
ylabel('$v_z$')
grid on 

%%
for i = 1:length(t)
    wt(i) = (x(i,11:13)*x(i,11:13)');
    vt(i) = (x(i,14:16)*x(i,14:16)');
    pt(i) = (pos(i,2:4)* pos(i,2:4)');
end
%%
figure(7)
subplot(2,1,1)
plot(t,wt)
grid on
xlabel('time (sec)')
ylabel('$\|\omega\|$') 
hold on
subplot(2,1,2)
plot(t,vt)
grid on
hold on
xlabel('time (sec)')
ylabel('$\|v\|$') 
%%
figure(11)
subplot(2,1,1)
hold on
plot(t,x(:,9).*x(:,1))
%plot(t,s(1:length(x),:).*x(:,1))
grid on
hold on
xlabel('time (sec)')
ylabel('$h\eta$')  

%%
%plot hybrid arc
%pos = 2*quatmultiply(q1,q2);
subplot(2,1,2)
hold on
plot(t,pt)
% hold on
% plot(t,pos(:,3))
% hold on
% plot(t,pos(:,4))
grid on
xlabel('time (sec)')
ylabel('$\|r_p\|^2$')
legend('$r_{p_x}$','$r_{p_y}$','$r_{p_z}$')
%legend('Full state measurement', 'Partial state measurement')


%%
%i = 1
for i = 1:length(t)
    %------------Noise----------%
%     tn = x(i,10);
%     tmp = abs(tnoise-tn);
%     [row col] = min(tmp);
%     pnoise = NOISEr(col);  % process noise
    %-------Controllerstates--------%
    etac = x(i,1);
    mu1c = x(i,2:4);
    eta2c = x(i,5);
    mu2c = x(i,6:8);
    wc = x(i,11:13);
    vc = x(i,14:16);
    %-----------GAINS--------%
    c = 0.5;
    kdw = 0.5;
    kdv = 0.5;
     h = x(i,9);
     %-------DISCONT CONTROLLER-------%
%     h = s(i);
    %-----Force caluclations-----%
    %issue with the dimension of tau (3x3), figure this out for the torque
    tau = -c*h*mu1c- kdw*wc;
    tauT(i) = tau*tau';
    F =  -(etac*mu2c-eta2c*mu1c-cross(mu1c,mu2c))-1*kdv*vc;
    FT(i) = F*F';
    
    clear etac mu1c eta2c mu2c wc vc 
end
Zt = sqrt(cumtrapz(t,tauT));
Zf = sqrt(cumtrapz(t,FT));
%
figure(8)
subplot(2,1,1)
plot(t,Zt)
grid on
hold on
xlabel('time (sec)')
ylabel('$\sqrt{\int_0^t\tau^\top \tau dt}$') 
subplot(2,1,2)
plot(t,Zf)
grid on
hold on
xlabel('time (sec)')
ylabel('$\sqrt{\int_0^tF^\top F dt}$') 
%%
figure(9)
 plot(t,x(:,9))
grid on
hold on
%-------DISCONT CONTROLLER-------%
%plot(t,s(1:length(x)))
xlabel('time (sec)')
ylabel('h') 
%% Principle angle
figure(10)
%  plot(t,180*ones(1,length(t)),'r','linewidth',2)
%  hold on
%  plot(t,-180*ones(1,length(t)),'r','linewidth',2)
hold on
 for i = 1:length(t)
     %Dcont
     %th = s(i)*2*acosd(x(i,1));
     %Hyd
     th = 2*acosd(x(i,1));%
 if th >= 180
     thn(i) = th -360;
 else
     thn(i) = th;
     
 end
 end
%     z
%plot(t,2*acosd(x(:,1)),'b')
%hold on
plot(t,thn)
grid on
hold on
xlabel('time (sec)')
ylabel('Principal angle of rotation $\theta$ degrees') 

%%
figure(12)
hold on
plot(t,2*acosd(x(:,1)))

