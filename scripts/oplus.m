function [pos_res, cov_res] = oplus(pos_a, pos_b, cov_a, cov_b, cov_a_b, cov_b_a)
    % if pos_b is a position
    if(size(pos_b, 1) == 2)
        R = [cos(pos_a(3)), -sin(pos_a(3));
             sin(pos_a(3)),  cos(pos_a(3))];
        pos_res = R*pos_b + pos_a(1:2);
        
        % if cov_a and cov_b are specified
        if(nargin >= 4)
            % if cov_a_b and and cov_b_a are not specified
            if(nargin == 4)
                cov_a_b = zeros(3,2);
                cov_b_a = zeros(2,3);
            end
            J = [1 0 -(pos_res(2)-pos_a(2)) cos(pos_a(3)) -sin(pos_a(3));
                 0 1  (pos_res(1)-pos_a(1)) sin(pos_a(3))  cos(pos_a(3));
                 0 0  1                     0              0            ];
             cov_res = J*[cov_a cov_a_b; cov_b_a cov_b]*J';
        else
            cov_res = nan;
        end
    % if pos_b is a pose
    elseif(size(pos_b, 1) == 3)
        R = [cos(pos_a(3)) -sin(pos_a(3)) 0;
             sin(pos_a(3))  cos(pos_a(3)) 0;
             0              0             1];
        pos_res = R*pos_b + pos_a;
        
        % if cov_a and cov_b are specified
        if(nargin >= 4)
            % if cov_a_b and and cov_b_a are not specified
            if(nargin == 4)
                cov_a_b = zeros(3,3);
                cov_b_a = zeros(3,3);
            end
            J = [1 0 -(pos_res(2)-pos_a(2)) cos(pos_a(3)) -sin(pos_a(3)) 0;
                 0 1  (pos_res(1)-pos_a(1)) sin(pos_a(3))  cos(pos_a(3)) 0;
                 0 0  1                     0              0             1];
             cov_res = J*[cov_a cov_a_b; cov_b_a cov_b]*J';
        else
            cov_res = nan;
        end
    else
        disp('error unvalide size(pos_b, 1)');
    end
end