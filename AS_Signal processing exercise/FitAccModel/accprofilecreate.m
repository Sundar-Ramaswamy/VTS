function xacc_gen=accprofilecreate(t,xacc,a0,jB,ItB,a1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% xacc_gen=accprofilecreate(t,xacc,a0,jB,ItB,a1)
%
% Description: The sub-function for fitting a three piece pice-wise linear
% model
%
% Indata:
%           t = time vector from original data (typically)
%           xacc = the acceleration vector from the original data
%           a0 = initial acceleration (fit-variable)
%           jB = The brake jerk (fit-variable)
%           ItB = The index of the start of braking (fit-variable)
%           a1 = The maximum deceleration (fit-variable)
%           
% Output: 
%       xacc_gen - a vector of acceleration based on time and input
%           variables
%
% Version   Date            Developer           Organisation
% 3.0       2017-11-01      Jonas Bärgman       Chalmers
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

    % Set delta-t
    dt=mean(diff(t));
    ITjB=length(xacc);
    
    % The first segment
    va1=a0 * ones(1,ItB);

    % The slope
    va2_tmp1=jB * [dt:dt:length(xacc)*dt];
    va2_tmp2=va2_tmp1+a0;
    
    % Cutoff at the final constant
    va2_tmp2(va2_tmp2<a1)=a1;
    va2=va2_tmp2;%-a0;

    % Produce the output
    xacc_gen=[va1 va2];
    xacc_gen=xacc_gen(1:length(xacc));
