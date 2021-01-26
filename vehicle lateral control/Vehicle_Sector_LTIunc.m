% Region of attraction analysis on the vehicle system with NN controller
clear

%% parameters
% Nominal speed of the vehicle travels at.
U = 28; % m/s
% Model
% Front cornering stiffness for one wheel.
Ca1 = -61595; % unit: Newtons/rad
% Rear cornering stiffness for one wheel.
Ca3 = -52095; % unit: Newtons/rad

% Front cornering stiffness for two wheels.
Caf = Ca1*2; % unit: Newtons/rad
% Rear cornering stiffness for two wheels.
Car = Ca3*2; % unit: Newtons/rad

% Vehicle mass
m = 1670; % kg
% Moment of inertia
Iz = 2100; % kg/m^2

% Distance from vehicle CG to front axle
a = 0.99; % m
% Distance from vehicle CG to rear axle
b = 1.7; % m

g = 9.81;

% sampling period
dt = 0.02;

% Continuous-time state space matrices
% States are lateral displacement(e) and heading angle error(deltaPsi) and 
% their derivatives.
% Inputs are front wheel angle and curvature of the road.
Ac = [0 1 0 0; ...
      0, (Caf+Car)/(m*U), -(Caf+Car)/m, (a*Caf-b*Car)/(m*U); ...
      0 0 0 1; ...
      0, (a*Caf-b*Car)/(Iz*U), -(a*Caf-b*Car)/Iz, (a^2*Caf+b^2*Car)/(Iz*U)];
B1c = [0; 
      -Caf/m;
      0; ...
      -a*Caf/Iz];
% discribes how the curvature enters the system. 
B2c = [0; 
      (a*Caf-b*Car)/m-U^2;
      0; ...
      (a^2*Caf+b^2*Car)/Iz];

%% x^+ = AG*x + BG1*q + BG2*usat
% XXX - You are using an Euler discretization below. This is probably fine
% for the example but there are better discretizations, e.g. ZOH. Note 
% that ZOH is more accurate for a larger freq range in the plots below.
%   Gc = ss(Ac,B1c,eye(4),0); 
%   Gd1 = ss(AG,BG1,eye(4),0,dt);   % Euler
%   Gd2 = c2d(Gc,dt);               % ZOH (Default for C2D)
%   i=1; figure(1); bode(Gc(i),'b',Gd1(i),'r--',Gd2(i),'k-.')
%   i=3; figure(2); bode(Gc(i),'b',Gd1(i),'r--',Gd2(i),'k-.')

AG = Ac*dt + eye(4);

% describes how q enters the system
BG1 = B1c*dt;
% describes how usat enters the system
BG2 = B1c*dt;
% number of nominal states
nG = size(AG, 1);
nu = 1;
nq = 1;
nusat = 1;

%  p = CG*xG + [DG1 DG2]*[q; usat] = usat
CG = zeros(nu,nG);
DG1 = zeros(nu,nu);
DG2 = eye(nu,nu);

%% load weights and biases of the NN controller     
fname = 'Wb_s32_tanh/';
% fname = './vehicle_Wb_s32_tanh/Wb_s32_tanh/';
load([fname 'W1.csv'])
load([fname 'W2.csv'])
load([fname 'W3.csv'])
n1 = size(W1,1);
n2 = size(W2,1);
n3 = size(W3,1);
nphi = n1+n2;

b1 = zeros(n1,1);
b2 = zeros(n2,1);
b3 = zeros(n3,1);

%% bounds for the inputs to the nonlinearity
xeq = zeros(4,1);
v1eq = W1*xeq + b1;
w1eq = tanh(v1eq);
v2eq = W2*w1eq + b2;
w2eq = tanh(v2eq);
v3eq = W3*w2eq + b3; % This is also u_*
% usat = sat(v3) = sat(u)

deltav1 = 0.6;
v1up = deltav1 + v1eq;
v1lb = v1eq - deltav1;

% 
alpha1 = min((tanh(v1up)-tanh(v1eq))./(v1up-v1eq), (tanh(v1eq)-tanh(v1lb))./(v1eq-v1lb));
beta = 1;
w1up = tanh(v1up);
w1lb = tanh(v1lb);

v2up = W2*1/2*(w1up+w1lb) + b2 + abs(W2)*1/2*abs(w1up-w1lb);
v2lb = W2*1/2*(w1up+w1lb) + b2 - abs(W2)*1/2*abs(w1up-w1lb);
alpha2 = min((tanh(v2up)-tanh(v2eq))./(v2up-v2eq), (tanh(v2eq)-tanh(v2lb))./(v2eq-v2lb));

w2up = tanh(v2up);
w2lb = tanh(v2lb);
v3up = W3*1/2*(w2up+w2lb) + b3 + abs(W3)*1/2*abs(w2up-w2lb);
v3lb = W3*1/2*(w2up+w2lb) + b3 - abs(W3)*1/2*abs(w2up-w2lb);
umax = 30/180*pi;
alpha3 = min((sat(v3up,umax,-umax)-sat(v3eq,umax,-umax))./(v3up-v3eq), (sat(v3eq,umax,-umax)-sat(v3lb,umax,-umax))./(v3eq-v3lb));

Alpha = blkdiag(diag(alpha1),diag(alpha2),alpha3);
Beta = beta*eye(nphi+nu);

% Filter for sector IQC
nphibar = nphi + nu;
Psi_phi = [Beta, -eye(nphibar);...
          -Alpha, eye(nphibar)];

%% multiplier for LTI uncertainty

delta_norm = 0.1;

Apsi = -eye(2);
nxi = size(Apsi,1);
Apsi = Apsi*dt + eye(nxi);
Bpsi1 = [1; 0];
Bpsi1 = Bpsi1*dt;
Bpsi2 = [0; 1];
Bpsi2 = Bpsi2*dt;
Cpsi = [0,1,0,0;0,0,0,1]';
Dpsi1 = [1,0,0,0]';
Dpsi2 = [0,0,1,0]';

% XXX Form Psi for debugging
% Psi = ss(Apsi,[Bpsi1 Bpsi2],[Cpsi],[Dpsi1 Dpsi2],dt);

%% construct the extended system
Abar = [AG, zeros(nG,nxi);...
        Bpsi1*CG, Apsi];
Bbar = [BG1, BG2;...
        Bpsi1*DG1+Bpsi2, Bpsi1*DG2];
Cbar = [Dpsi1*CG, Cpsi];
Dbar = [Dpsi1*DG1+Dpsi2, Dpsi1*DG2];
nzeta = nG + nxi;

%% Convex Optimization - compute ROA
cvx_begin sdp quiet
    cvx_solver mosek
    
    % Variables
    variable P(nzeta,nzeta) symmetric;
    variable Tphi(nphibar,nphibar) diagonal; 
    P >= 1e-8*eye(nzeta);
    Tphi(:) >= 0;
    
    % XXX - Odd behavior
    if true
        % Original Code
        variable M11(2,2) symmetric;
        M11 >= 0;
    else
        % Restrict M11 to choose the constant term in Psi
        % (This corresponds to assuming uncertainty is NLTV)
        % This increases the trace(P) cost (as expected)
        % but also seems to increase the radii of the
        % reachable set (which I guess can happen but is
        % a bit unexpected).
        variable lambda1;
        lambda1>=0;
        M11 = diag([lambda1 0]); 
    end
    M = blkdiag(delta_norm*M11, -1/delta_norm*M11);
    
    % Term to bound nonlinearity Phi
    Mphi = [zeros(nphibar), Tphi; Tphi, zeros(nphibar)];
    Rphi = [W1, zeros(n1, nxi+nphibar+nq);...
            zeros(n2, nzeta), W2, zeros(n2,n2+nusat+nq);...
            zeros(n3, nzeta+n1),W3,zeros(n3,nusat+nq);...
            zeros(nphibar,nzeta),eye(nphibar), zeros(nphibar, nq)];       
    Qphi = Rphi'*Psi_phi'*Mphi*Psi_phi*Rphi;
    
    % Term to represent V(k+1) - V(k)
    % XXX - This seems fine but it is odd that you use the order 
    % [xG; xPsi] when defining Abar but appear to use the reverse ordering
    % below when you define Rnext.
    S = [Abar'*P*Abar - P, Abar'*P*Bbar;...
         Bbar'*P*Abar, Bbar'*P*Bbar];
    RV = [eye(nzeta), zeros(nzeta,nphibar + nq);...
          zeros(nq, nzeta + nphibar), eye(nq);...
          zeros(nu, nzeta + n1 + n2), eye(nu), zeros(nu, nq)];
    QV = RV'*S*RV;
    
    % Term that uses IQC
    Qpi = RV'*[Cbar Dbar]'*M*[Cbar Dbar]*RV;
    
    % Matrix Inequality
    % basis vector [zeta; w_phibar; q]
    % w_phibar = [w1; w2; usat]
    % usat = 
    QV + Qphi + Qpi <= 1e-8*eye(nzeta+nphibar+nq);
    for i = 1:n1
    % enforce {x: (zeta-zetaeq)'P(zeta-zetaeq)<=1} \subset {x: |W1(i,:)*(x-xeq)| <= r1} 
        [deltav1^2, [W1(i,:) zeros(1,nxi)];...
         [W1(i,:) zeros(1,nxi)]', P] >=0;...
    end
    minimize(trace(P(1:nG,1:nG)))
cvx_end

if ~isnan(P)
    radii = 1./sqrt(eig(P(1:nG,1:nG)))
    traceP = trace(P(1:nG,1:nG))
    volume = det(inv(P(1:nG,1:nG)))
    
    %
%     pvar x1 x2 x3 x4
%     x = [x1;x2;x3;x4];
%     V = x'*P(1:nG,1:nG)*x;
% 
%     subplot(1,2,1)
%     domain1 = [-40, 40, -10, 10];
%     pcontour(subs(V,[x3;x4],[0;0]),1,domain1,'--r')
%     xlim(domain1(1,1:2))
%     ylim(domain1(1,3:4))
%     hold on
% 
%     subplot(1,2,2)
%     domain2 = [-0.6, 0.6, -6, 6];
%     pcontour(subs(V,[x1;x2],[0;0]),1,domain2,'--r')
%     hold on
%     xlim(domain2(1,1:2))
%     ylim(domain2(1,3:4))
else
    disp('Infeasible');
end