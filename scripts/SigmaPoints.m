function [samples, weights, n_samples] = SigmaPoints(x, P, w0)
    
    n = length(x);
    n_samples = 2*n+1;
    if nargin<3
        w0 = 1/n_samples;
    end
    [V, D] = eig(P);
    sDV = sqrt(n/(1-w0))*sqrt(diag(D)').*V;
    samples = repmat(x,1,n_samples)+[zeros(n,1) sDV -sDV];
    weights = (1-w0)/(2*n)*ones(1,n_samples);
    weights(1) = w0;
end