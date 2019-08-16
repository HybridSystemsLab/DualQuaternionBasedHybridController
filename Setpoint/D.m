function v  = D(x) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by BMalladi)
%
% Project: Simulation of a hybrid system (Rigid body attitude)
%
% Name: D.m
%
% Description: Jump set
%
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
global delta
xsys = x(1:9);
eta = xsys(1);
h = xsys(8);


if (h*eta +delta) <= 1e-6
   v = 1;
 else
   v = 0;
end
 









    