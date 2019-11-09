function v  = C2(x) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab Function  Author: Ricardo Sanfelice (Revised by Giampiero Campa)
%
% Project: Simulation of a hybrid system (Bouncing ball)
%
% Name: C.m
%
% Description: Flow set
%
% Version: 1.0
% Required files: - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
global delta
 xsys = x(1:9);
 h = xsys(9);
 eta = xsys(1);
 
if (h*eta +delta) >= 1e-6 
    v = 1;
else
    v = 0;
end

%------Discontinuous law

% if eta >= 0 
%     v = 1;
% else
%     v = 0;
% end

