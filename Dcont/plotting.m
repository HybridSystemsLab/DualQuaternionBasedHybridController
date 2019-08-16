%%
j =1;
for i = 1:1:length(t)
    q1(j,1:4) = [x(i,1) -x(i,2:4)];
    q2(j,1:4) = x(i,5:8);
    ti = t(i);
    j = j+1;
end

%plot hybrid arc
figure(114)
 pos = 2*quatmultiply(q1,q2);
subplot(2,1,1)
plot(t(i),pos(:,2))
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
for i = 1:length(t)
    wt(i) = (x(i,11:13)*x(i,11:13)');
    vt(i) = (x(i,14:16)*x(i,14:16)');
    pt(i) = (pos(i,2:4)* pos(i,2:4)');
end
%%
figure(1)
subplot(2,1,1)
hold on
% plot(t,x(:,9).*x(:,1))
plot(t,s(1:length(x),:).*x(:,1))
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
grid on
xlabel('time (sec)')
ylabel('$\|r_p\|^2$')
%legend('$r_{p_x}$','$r_{p_y}$','$r_{p_z}$')
%legend('Full state measurement', 'Partial state measurement')


%% Principle angle
figure(2)
%  plot(t,180*ones(1,length(t)),'r','linewidth',2)
%  hold on
%  plot(t,-180*ones(1,length(t)),'r','linewidth',2)
hold on
 for i = 1:length(t)
     %Dcont
     th = s(i)*2*acosd(x(i,1));
     %Hyd
     %th = 2*acosd(x(i,1));%
 if th >= 180
     thn(i) = (th -360);
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
figure(7)
subplot(2,1,1)
plot(t,wt)
grid on
xlabel('time (sec)')
ylabel('$\|\omega\|^2$') 
hold on
subplot(2,1,2)
plot(t,vt)
grid on
hold on
xlabel('time (sec)')
ylabel('$\|v\|^2$') 
%%
%i = 1
for i = 1:length(t)
    %------------Noise----------%
    tn = x(i,10);
    tmp = abs(tnoise-tn);
    [row col] = min(tmp);
    pnoise = NOISEr(col);  % process noise
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
%    h = s(i);
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
%%
figure(4)
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
%-----------PLOT ETA---------%
subplot(2,1,1)
hold on
%plot(t,etar(1:length(x)))
grid on
hold on
plot(t,x(1:length(x)),'r')
%xlabel('time (sec)')
ylabel('$\eta$')
hold on
%-------------PLOT CHATTERING-------%
subplot(2,1,2)
plot(t,s(1:length(x)))
% plot(t,x(:,9))
hold on
grid on
ylabel('Switching logic $h$')
xlabel('time (sec)')
%% NEW PLOT FOR POSITION, LINEAR/ANGULAR VELOCITIES
figure(10)
subplot(3,1,1)
hold on
plot(t,pt)
grid on
xlabel('time (sec)')
ylabel('$\|r_p\|^2$')
subplot(3,1,2)
plot(t,wt)
grid on
xlabel('time (sec)')
ylabel('$\|\omega\|^2$') 
hold on
subplot(3,1,3)
plot(t,vt)
grid on
hold on
xlabel('time (sec)')
ylabel('$\|v\|^2$') 

