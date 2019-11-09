function xplus = g2(x)

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
xsys = x(1:10);
sysp = xsys(1:8);
h = xsys(9);
eta = xsys(1);
tau = xsys(10);
wsp = x(11:16);

if (h*eta +delta) <= 1e-6
    hplus = -h;
else
     hplus = h;
end

% if eta < 0
%     hplus = -h;
% end

xplus = [sysp;hplus;tau;wsp];


