function [x,u] = nnclosedloop(Nstep,x0,W,b,g,m,l,mu,dt)
% Simulate system for N steps from the initial condition x0.
%
% Syntax for linear plant: x(k+1) = Ad x(k) + Bd u(k)
%   [x,u] = nnclosedloop(N,x0,W,b,Ad,Bd)  
% Syntax for nonlinear plant: x(k+1) = f(x(k),u(k)) 
%   [x,u] = nnclosedloop(N,x0,W,b,fh)  
%   where fh is a function handle for f.

% Initialize outputs
u0 = LOCALnn(x0,W,b);
Nu = numel(u0);
Nx = numel(x0);

x = zeros(Nx,Nstep);
x(:,1) = x0;

u = zeros(Nu,Nstep);
u(:,1) = u0;

% Simulate System
% nin = nargin;
for i=2:Nstep
%     if nin==6
      x(:,i) = x(:,i-1)+[x(2,i-1);...
                           g/l*sin(x(1,i-1))-mu/(m*l^2)*x(2,i-1)+1/(m*l^2)*u(:,i-1)]*dt;
%         x(:,i) = Ad*x(:,i-1)+Bd*u(:,i-1);
%     else
%         fh = Ad;
%         x(:,i) = fh( x(:,i-1), u(:,i-1) );
%     end
    
    u(:,i) = LOCALnn(x(:,i),W,b);
end


%%
% LOCAL Function to evaluate the Neural Network
function u = LOCALnn(x,W,b)
    
Nlayer = numel(W);
z = x;
for i=1:(Nlayer-1)
    z = W{i}*z + b{i};
    
    % XXX - Nonlinearity could be input as a function handle
    z = tanh(z);
end
u = W{end}*z+b{end};
    

