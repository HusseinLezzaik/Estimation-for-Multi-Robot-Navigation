function [pos_res, cov_res] = oplusUT(pos_a, pos_b, cov_a, cov_b, w0, cov_a_b, cov_b_a)
    % if pos_b is a position
    if(size(pos_b, 1) == 2)
        
        % if cov_a and cov_b are specified
        if(nargin >= 4)
            transf = @(x) [cos(x(3)) -sin(x(3));
                           sin(x(3))  cos(x(3))]*x(4:5)+x(1:2);
            pos_in = [pos_a;pos_b];
            
            % if cov_a_b and and cov_b_a are not specified
            if(nargin == 4 | nargin == 5)
                cov_a_b = zeros(3,2);
                cov_b_a = zeros(2,3);
            end
            
            cov_in = [cov_a cov_a_b; cov_b_a cov_b];
            
            if nargin == 4 | nargin == 6
                [samples, weights, n_samples] = SigmaPoints(pos_in, cov_in);
            else
                [samples, weights, n_samples] = SigmaPoints(pos_in, cov_in, w0);
            end

            images = zeros(2,n_samples);
            for i=1:n_samples
                images(:,i) = transf(samples(:,i));
            end
            
            pos_res = zeros(2,1);
            for i=1:n_samples
                pos_res = pos_res + weights(i)*images(:,i);
            end
            cov_res = zeros(2,2);
            for i=1:n_samples
                cov_res = cov_res + weights(i)*((images(:,i) - pos_res)*(images(:,i) - pos_res)');
            end
                        
        else
            disp("Warning: Unscented transform will not be used: no specified covariances");
            [pos_res, cov_res] = oplus(pos_a, pos_b);
        end
    % if pos_b is a pose
    elseif(size(pos_b, 1) == 3)
         % if cov_a and cov_b are specified
        if(nargin >= 4)
            transf = @(x) [cos(x(3)) -sin(x(3))  0;
                           sin(x(3))  cos(x(3))  0;
                              0              0   1]*x(4:6)+x(1:3);
            pos_in = [pos_a;pos_b];
            
            % if cov_a_b and and cov_b_a are not specified
            if(nargin == 4 | nargin == 5)
                cov_a_b = zeros(3,3);
                cov_b_a = zeros(3,3);
            end
            
            cov_in = [cov_a cov_a_b; cov_b_a cov_b];
            
            if nargin == 4 | nargin == 6
                [samples, weights, n_samples] = SigmaPoints(pos_in, cov_in);
            else
                [samples, weights, n_samples] = SigmaPoints(pos_in, cov_in, w0);
            end

            images = zeros(3,n_samples);
            for i=1:n_samples
                images(:,i) = transf(samples(:,i));
            end

            pos_res = zeros(3,1);
            for i=1:n_samples
                pos_res = pos_res + weights(i)*images(:,i);
            end
            cov_res = zeros(3,3);
            for i=1:n_samples
                cov_res = cov_res + weights(i)*((images(:,i) - pos_res)*(images(:,i) - pos_res)');
            end
                        
        else
            disp("Warning: Unscented transform will not be used: no specified covariances");
            [pos_res, cov_res] = oplus(pos_a, pos_b);
        end
    else
        disp('error unvalide size(pos_b, 1)');
    end
end

% function [pos_res, cov_res] = oplusUT(pos_a, pos_b, cov_a, cov_b, cov_a_b, cov_b_a)
%     % if pos_b is a position
%     if(size(pos_b, 1) == 2)
%         
%         % if cov_a and cov_b are specified
%         if(nargin >= 4)
%             transf = @(x) [cos(x(3)) -sin(x(3));
%                            sin(x(3))  cos(x(3))]*x(4:5,:)+x(1:2,:);
%             pos_in = [pos_a;pos_b];
%             
%             % if cov_a_b and and cov_b_a are not specified
%             if(nargin == 4)
%                 cov_a_b = zeros(3,2);
%                 cov_b_a = zeros(2,3);
%             end
%             
%             cov_in = [cov_a cov_a_b; cov_b_a cov_b];
%             
%             [samples, weights, n_samples] = SigmaPoints(pos_in, cov_in);
% 
%             images = zeros(2,n_samples);
%             for i=1:n_samples
%                 images(:,i) = transf(samples(:,i));
%             end
% 
%             pos_res = sum(weights.*images, 2);
%             cov_res = (sqrt(weights).*(images - pos_res))*(sqrt(weights).*(images - pos_res))';
%                         
%         else
%             disp("Warning: Unscented transform will not be used: no specified covariances");
%             [pos_res, cov_res] = oplus(pos_a, pos_b);
%         end
%     % if pos_b is a pose
%     elseif(size(pos_b, 1) == 3)
%          % if cov_a and cov_b are specified
%         if(nargin >= 4)
%             transf = @(x) [cos(x(3)) -sin(x(3))  0;
%                            sin(x(3))  cos(x(3))  0;
%                               0              0   1]*x(4:6,:)+x(1:3,:);
%             pos_in = [pos_a;pos_b];
%             
%             % if cov_a_b and and cov_b_a are not specified
%             if(nargin == 4)
%                 cov_a_b = zeros(3,3);
%                 cov_b_a = zeros(3,3);
%             end
%             
%             cov_in = [cov_a cov_a_b; cov_b_a cov_b];
%             
%             [samples, weights, n_samples] = SigmaPoints(pos_in, cov_in);
% 
%             images = zeros(3,n_samples);
%             for i=1:n_samples
%                 images(:,i) = transf(samples(:,i));
%             end
% 
%             pos_res = sum(weights.*images, 2);
%             cov_res = (sqrt(weights).*(images - pos_res))*(sqrt(weights).*(images - pos_res))';
%                         
%         else
%             disp("Warning: Unscented transform will not be used: no specified covariances");
%             [pos_res, cov_res] = oplus(pos_a, pos_b);
%         end
%     else
%         disp('error unvalide size(pos_b, 1)');
%     end
% end