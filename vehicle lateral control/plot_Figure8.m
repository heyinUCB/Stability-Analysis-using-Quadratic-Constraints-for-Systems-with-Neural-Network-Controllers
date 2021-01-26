% Poly = Polyhedron('A',[W0;-W0],'b',r1*ones(nz1*2,1));
% plot(Poly,'alpha',0.1)
% hold on

%%
pvar x1 x2 x3 x4
x = [x1;x2;x3;x4];
V = (x-xeq)'*P(1:nG,1:nG)*(x-xeq);

subplot(1,2,1)
domain1 = [-50, 50, -10, 10];
[C,h] = pcontour(subs(V,[x3;x4],xeq(3:4)),1,domain1);
h.LineColor = mycolor('coolblue');
h.LineWidth = 4;
hold on
plot(xeq(1),xeq(2),'x','MarkerSize',10,'color',mycolor('maroon'))
grid on
x1g = linspace(domain1(1),domain1(2),1e3);
figure(1)
for i=1:n1
    plot(x1g,(deltav1-W1(i,1)*(x1g-xeq(1))+W1(i,2)*xeq(2))/W1(i,2),'color',mycolor('orange')), 
    hold on
    plot(x1g,(-deltav1-W1(i,1)*(x1g-xeq(1))+W1(i,2)*xeq(2))/W1(i,2),'color',mycolor('orange'))
    hold on;
end

ah = [];
ah(1)=annotation('textarrow',[0.33 0.37],[0.85 0.65],'String','$\{x:\underline{v}^1 \leq v^1 \leq \bar{v}^1\}$','interpreter','latex');
set(ah,'FontSize',18)

xlim(domain1(1,1:2))
ylim(domain1(1,3:4))
xlabel('$e$','interpreter','latex')
ylabel('$\dot{e}$','interpreter','latex')

subplot(1,2,2);
domain2 = [-0.7, 0.7, -10, 10];
[C,h] = pcontour(subs(V,[x1;x2],xeq(1:2)),1,domain2,'k');
h.LineColor = mycolor('coolblue');
h.LineWidth = 4;
hold on
plot(xeq(3),xeq(4),'x','MarkerSize',10,'color',mycolor('maroon'))
grid on
x3g = linspace(domain2(1),domain2(2),1e3);
figure(1)
for i=1:n1
    plot(x3g,(deltav1-W1(i,3)*(x3g-xeq(3))+W1(i,4)*xeq(4))/W1(i,4),'color',mycolor('orange')), 
    hold on
    plot(x3g,(-deltav1-W1(i,3)*(x3g-xeq(3))+W1(i,4)*xeq(4))/W1(i,4),'color',mycolor('orange'))
    hold on;
end
xlim(domain2(1,1:2))
ylim(domain2(1,3:4))
xlabel('$e_{\theta}$','interpreter','latex')
ylabel('$\dot{e}_{\theta}$','interpreter','latex')

garyfyFigure