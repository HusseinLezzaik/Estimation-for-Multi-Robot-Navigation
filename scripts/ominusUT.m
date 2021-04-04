function [pos_out, cov_out] = ominusUT(pos_in, cov_in,w0)
    % if cov_in is specified
    if nargin >=2
        transf = @(x)  [ cos(x(3))  sin(x(3))    0;
                        -sin(x(3))  cos(x(3))    0;
                             0          0        1]*(-x);

        if nargin == 2
            [samples, weights, n_samples] = SigmaPoints(pos_in, cov_in);
        else
            [samples, weights, n_samples] = SigmaPoints(pos_in, cov_in, w0);
        end
        
        images = zeros(3,n_samples);
        for i=1:n_samples
            images(:,i) = transf(samples(:,i));
        end
        
        pos_out = sum(weights.*images, 2);
        cov_out = (sqrt(weights).*(images - pos_out))*(sqrt(weights).*(images - pos_out))';
        
    else
        disp("Warning: Unscented transform will not be used: no specified covariances");
        [pos_out, cov_out] = ominus(pos_in);
    end

end