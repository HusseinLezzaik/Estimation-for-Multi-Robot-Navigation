function [x,P,omega]=CI_update(x1,P1,x2,P2,C,omega)
    %CI data fusion of two estimates or a prediction with a measurement
    %If omega (0<omega<1)is omitted then the routine computes the ellipsoid 
    %that has the minimum volume
    P1i=P1^-1;
    P2i=P2^-1;

    if nargin < 6 %compute omega
        f=inline('1/det(P1i*omega+C*P2i*C*(1-omega))','omega', 'P1i', 'P2i', 'C');
        %Work out omega using the matlab constrained minimizer function
        %omega=fminbnd(f,0,1,optimset('Display','off'),P1i,P2i,C);
        %The unconstrained version of this optimization is
        omega=fminsearch(f,0.5,optimset('Display','off'),P1i,P2i,C);
        omega=min(max(omega,0),1); %saturation
       
    end

    P=(P1i*omega+C'*P2i*C*(1-omega))^-1;% New covariance
    nu=x2-C*x1;%innovation
    nu(3) = wrap_angle(nu(3));
%     disp(['Gain CI = ' ]);
    K=(1-omega)*P*C'*P2i; %Gain

    x=x1+K*nu;% New estimate

end