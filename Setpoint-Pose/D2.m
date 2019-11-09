function v  = D2(x) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by Giampiero Campa)
%
% Project: Simulation of a hybrid system (Bouncing ball)
%
% Name: D.m
%
% Description: Jump set
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
global delta
 xsys = x(1:9);
 h = xsys(9);
 eta = xsys(1);

if (h*eta +delta) <= 1e-6
   v = 1;
 else
   v = 0;
end
 
%------Discountinuous law

% if eta < 0
%    v = 1;
%  else
%    v = 0;
%  end








    