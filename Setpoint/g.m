function xplus = g(x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by BMalladi)
%
% Project: Simulation of a hybrid system (Rigid body attitude)
%
% Name: g.m
%
% Description: Jump map
%
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global delta
xsys = x(1:9);
eta = xsys(1);
h = xsys(8);

if (h*eta +delta) <= 1e-6
    hplus = -h;
else
     hplus = h;
end

sysp = xsys(1:4);
wsp = x(5:7);
taup = xsys(9);

xplus = [sysp;wsp;hplus;taup];


