function v  = Ctk(x) 

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
 h = x(29);
qrd = x(1:4);
qrb = x(15:18);
barq = qnpdt(quatinv(qrd')',qrb);
 eta = barq(1);
 
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

