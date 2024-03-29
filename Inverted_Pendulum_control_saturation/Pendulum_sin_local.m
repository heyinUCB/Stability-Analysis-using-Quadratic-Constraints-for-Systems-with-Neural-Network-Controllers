% Region of attraction analysis on the pendulum system with NN controller
clear

%% parameters
g = 10; % gravitational coefficient
m = 0.15; % mass
l = 0.5; % length
mu = 0.05; % frictional coefficient
dt = 0.02; % sampling period

%% x^+ = AG*x + BG1*q + BG2*u
AG = [1,      dt;...
      g/l*dt, 1-mu/(m*l^2)*dt];
% describes how q enters the system
BG1 = [0;...
       -g/l*dt];
% describes how u enters the system
BG2 = [0;...
      dt/(m*l^2)];
nG = size(AG, 1);
nu = 1;
nq = 1;

%  v_Delta = CG*xG + [DG1 DG2]*[q; u] = xG
CG = [1, 0];
DG1 = 0;
DG2 = 0;

%% load weights and biases of the NN controller     
%fname = '../vehicle_training/Wb_s32_tanh/';
fname = 'Wb_s32_tanh/';
load([fname 'W1.csv'])
load([fname 'W2.csv'])
load([fname 'W3.csv'])
n1 = size(W1,1);
n2 = size(W2,1);
n3 = size(W3,1);
nphi = n1+n2+n3;

b1 = zeros(n1,1);
b2 = zeros(n2,1);
b3 = zeros(n3,1);

%% bounds for the inputs to the nonlinearity
xeq = [0.0; 0.0];
v1eq = W1*xeq + b1;
w1eq = tanh(v1eq);
v2eq = W2*w1eq + b2;
w2eq = tanh(v2eq);
v3eq = W3*w2eq + b3; % This is also u_*
% usat = sat(v3) = sat(u)

deltav1 = 0.1;
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
umax = 0.7;
alpha3 = min((sat(v3up,umax,-umax)-sat(v3eq,umax,-umax))./(v3up-v3eq), (sat(v3eq,umax,-umax)-sat(v3lb,umax,-umax))./(v3eq-v3lb));

Alpha = blkdiag(diag(alpha1),diag(alpha2),alpha3);
Beta = beta*eye(nphi);

% Filter for sector IQC
Psi_phi = [Beta, -eye(nphi);...
          -Alpha, eye(nphi)];

%% Define IQCs for the nonlinearity x1 - sin(x1)
% x1 - sin(x1) is slope-restricted in [0, 2] globally ---> off-by-one IQC
% x1 - sin(x1) is slope-restricted in [0, 0.1224] locally n x1 in [-0.5, 0.5] ---> off-by-one IQC

% x1 - sin(x1) is sector-bounded in [0, 1.22] globally ---> sector IQC
% x1 - sin(x1) is sector-bounded in [0, 1] locally on x1 in [-pi,pi]
% x1 - sin(x1) is sector-bounded in [0, 0.7606] locally on x1 in [-2.5,2.5]
% x1 - sin(x1) is sector-bounded in [0, 0.0411] locally on x1 in [-0.5,0.5]
% define the filter for off-by-one IQC
x1bound = 0.73;
L_slope = 1 - cos(x1bound);
m_slope = 0;
Apsi = 0;
npsi = size(Apsi,1);
Apsi = Apsi*dt + eye(npsi);
Bpsi1 = -L_slope;
Bpsi2 = 1;
Bpsi1 = Bpsi1*dt;
Bpsi2 = Bpsi2*dt;
Cpsi = [1; 0];
Dpsi1 = [L_slope; -m_slope];
Dpsi2 = [-1; 1];
% M matrix for off-by-one IQC      
M_off = [0, 1;...
         1, 0];
     
% define the filter for sector IQC
L_sector = (x1bound - sin(x1bound))/x1bound;
m_sector = 0;
Psi_sec = [L_sector, -1;...
           -m_sector,    1];
% M matrix for sector IQC
M_sec = [0, 1;...
         1, 0];
     
%% construct the extended system
Abar = [AG, zeros(nG,npsi);...
        Bpsi1*CG, Apsi];
Bbar = [BG1, BG2;...
        Bpsi1*DG1+Bpsi2, Bpsi1*DG2];
Cbar = [Dpsi1*CG, Cpsi];
Dbar = [Dpsi1*DG1+Dpsi2, Dpsi1*DG2];
nzeta = nG + npsi;

Rzeta = [eye(nzeta), zeros(nzeta, nphi+nq)];
Rq = [zeros(nq,nzeta+nphi), eye(nq)];
Rusat = [zeros(nu, nzeta+n1+n2), eye(nu), zeros(nu,nq)];
Rwphi = [zeros(nphi,nzeta), eye(nphi), zeros(nphi,nq)];
Rvphi = [blkdiag([W1,zeros(n1,npsi)], W2, [W3,zeros(n3,nu)]),zeros(nphi,nq)];
Rp = [1, zeros(1, nzeta-1+nphi+nq)];

%% Convex Optimization - compute ROA
cvx_begin sdp quiet
    cvx_solver mosek
    
    % Variables
    variable P(nzeta,nzeta) symmetric;
    variable Tphi(nphi,nphi) diagonal; 
    variable lambda1;
    variable lambda2;
    lambda1 >= 0;
    lambda2 >= 0;
    Tphi(:) >= 0;
    P >= 1e-8*eye(nzeta);

    % Term to bound nonlinearity Phi
    Mphi = [zeros(nphi), Tphi; Tphi, zeros(nphi)];    
    Rphi = [Rvphi; Rwphi];
    Qphi = Rphi'*Psi_phi'*Mphi*Psi_phi*Rphi;
    
    % Term to represent V(k+1) - V(k)
    S = [Abar'*P*Abar - P, Abar'*P*Bbar;...
         Bbar'*P*Abar, Bbar'*P*Bbar];
    RV = [Rzeta; Rq; Rusat];
    QV = RV'*S*RV;
    
    % Term that uses off-by-one IQC to bound the nonlinearity Delta
    Qoff = lambda2*RV'*[Cbar Dbar]'*M_off*[Cbar Dbar]*RV;
    
    % Term that uses sector IQC to bound the nonlinearity Delta
    R_sec = [Rp;...
             Rq];
    Qsec = lambda1*R_sec'*Psi_sec'*M_sec*Psi_sec*R_sec;
    
    % Matrix Inequality
    QV + Qphi + Qsec + Qoff <= -1e-10*eye(nzeta+nphi+nq);
    for i = 1:n1
    % enforce {x: x'Px<=1} \subset {x: |[W0(i,:) 0]*x| <= r1} 
        [deltav1^2, [W1(i,:) zeros(1,npsi)];...
         [W1(i,:) zeros(1,npsi)]', P] >=0;...
    end
    % enforce {x: x'Px<=1} \subset {x: |x1| <= x1bound}
    [x1bound^2, [1,0, zeros(1,npsi)];...
     [1,0, zeros(1,npsi)]', P] >=0;...
    minimize(trace(P(1:nG,1:nG)))
cvx_end
P
if ~isnan(P)
    radii = 1./sqrt(eig(P(1:nG,1:nG)))
    traceP = trace(P(1:nG,1:nG))
end

%% plot results
% Simulation results
N1 = 80;
x1box = linspace(-2,2,N1);
N2 = 80;
x2box = linspace(-2,2,N2);
xIC = [x1box x1box  x1box(1)*ones(1,N1) x1box(end)*ones(1,N1); ...
    x2box(1)*ones(1,N2) x2box(end)*ones(1,N2) x2box x2box];
Nstep = 200;
for i=1:size(xIC,2)
    x0 = xIC(:,i);
    [x,u] = nnclosedloop(Nstep,x0,{W1,W2,W3},{b1,b2,b3},g,m,l,mu,dt,umax);
    if abs(x(:,end)) <= 0.5
        plot(x(1,:),x(2,:),'g');
    else
        plot(x(1,:),x(2,:),'r');
    end
    hold on
end

%% Hyperplanes 
x1 = linspace(-10,10,1e3);
figure(1)
for i=1:n1
    plot(x1,(deltav1-W1(i,1)*x1)/W1(i,2),'color',mycolor('orange')) 
    plot(x1,(-deltav1-W1(i,1)*x1)/W1(i,2),'color',mycolor('orange'))
    hold on;
end
plot([x1bound,x1bound],[-9,9],'color',mycolor('maroon'))
plot([-x1bound,-x1bound],[-9,9],'color',mycolor('maroon'))

% ROA
pvar x1 x2
V = [x1,x2]*P(1:nG,1:nG)*[x1;x2];
domain1 = [-10, 10, -10, 10];
[C,h] = pcontour(V,1,domain1,'r',[300, 300]);
h.LineColor = mycolor('coolblue');
h.LineWidth = 4;
hold on

% xeq
plot(xeq(1),xeq(2),'kx','MarkerSize',10);
hold off;

ah = [];
ah(1)=annotation('textarrow',[0.3018 0.2786],[0.8286 0.6262],'String','$\{x:\underline{v}^1 \leq v^1 \leq \bar{v}^1\}$','interpreter','latex');
ah(2)=annotation('textarrow',[0.43 0.38],[0.18 0.25],'String','$\{x:\underline{\theta} \leq \theta \leq \bar{\theta}\}$','interpreter','latex');
set(ah,'FontSize',18)

grid on;
axis([-2 2 -2 2]);
xlabel('$\theta$','interpreter','latex')
ylabel('$\dot{\theta}$','interpreter','latex')

garyfyFigure