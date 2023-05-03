function cons_di_selffeedback_1
%agent dynamics
 A=[0 1;
    0 0];
 B=[0 1]';
% %network topology-6 agent system  
L=[1 0 -1 0 0 0;
    -1 1 0 0 0 0;
    -1 0 1 0 0 0;
    0 -1 0 1 0 0;
    0 0 -1 0 1 0;
    0 0 0 -1 -1 2];
% %local controller
 gamma=5;
 K=[1 gamma];
%self_feedback
z=[0 0 0 0 0 0]';
       %Dz=diag(z);
       Dz=0*eye(6);
       Ks=5*[1 gamma];
%solve ode
tspan=[0 25];
x10=[1 -5]';
x20=[2 1]';
x30=[-2 -1]';
x40=[5 1]';
x50=[-4 -0.5]';
x60=[3 0]';

x0=[x10;x20;x30;x40;x50;x60];
[t,y]=ode45(@cons_di_dyn1,tspan,x0);

%plot the rsults
figure;
%plot(t,y(:,1),'r','Linewidth',1.5);
hold on
% plot(t,y(:,3),'r','Linewidth',1.5);
% plot(t,y(:,5),'r','Linewidth',1.5);
% plot(t,y(:,7),'r','Linewidth',1.5);
% plot(t,y(:,9),'r','Linewidth',1.5);
% plot(t,y(:,11),'r','Linewidth',1.5);
plot(t,y(:,2),'r','Linewidth',1.5);
plot(t,y(:,4),'b','Linewidth',1.5);
 plot(t,y(:,6),'r','Linewidth',1.5);
plot(t,y(:,8),'b','Linewidth',1.5);
plot(t,y(:,10),'r','Linewidth',1.5);
plot(t,y(:,12),'b','Linewidth',1.5);
title('DI-self feedback')
xlabel('Time');ylabel('Magnitude');
leg=legend('$x_{11}$','$x_{21}$','$x_{31}$','$x_{41}$','$x_{51}$');
set(leg,'Interpreter','latex','Fontsize',10);
hold off
%closed loop dynamics
    function dxdt=cons_di_dyn1(t,x)
        dxdt=(kron(eye(6),A)-kron(Dz,B*Ks)-kron(L,B*K))*x+kron(Dz,B*Ks)*x0;
    end
end