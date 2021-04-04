function [pos_out, cov_out] = ominus(pos_in, cov_in)
    R = [cos(pos_in(3)) sin(pos_in(3)) 0;
         -sin(pos_in(3))  cos(pos_in(3)) 0;
         0              0             1];
    pos_out = R*(-pos_in);
    
    % if cov_in is specified
    if(nargin >= 2)
        J = [-cos(pos_in(3)) -sin(pos_in(3))  pos_out(2);
              sin(pos_in(3)) -cos(pos_in(3)) -pos_out(1);
              0               0              -1];
         cov_out = J*cov_in*J';
    else
        cov_out = nan;
    end
end