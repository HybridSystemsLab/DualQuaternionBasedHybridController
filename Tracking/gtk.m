function xplus = gtk(x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by Giampiero Campa)
%
% Project: Simulation of a hybrid system (Bouncing ball)
%
% Name: g.m
%
% Description: Jump map
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global delta
xsysp = x(1:28);
h = x(29);
tau = x(30);

%eta = x(15);
qrd = x(1:4);
qrb = x(15:18);
barq = qnpdt(quatinv(qrd')',qrb);
eta = barq(1);

if (h*eta +delta) <= 1e-6
    hplus = -h;
else
     hplus = h;
end

% if eta < 0
%     hplus = -h;
% end

xplus = [xsysp;hplus;tau];


