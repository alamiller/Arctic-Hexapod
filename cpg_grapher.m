close all

% initialise state variables
%x=ones(6,6);
y=ones(6,6);

% sample_time=0.025;
sample_time = 0.157;
time_end = 2;

r=1;
mu=1;
w= 10;
t= 0:sample_time:time_end;
s=length(t);

z=ones(6,s);

% coupling_strength=0.5;

for i = 2:6
   z(i,:)= i;
    
end

alpha=10;
beta=1000;

% k0 to 04 value
% k=[90.0, 30, 30, -1.0, 1.0];
k=[90.0, 30, 30, -0.1, 0.1];
b=[90.0, 60.0,40.0, 90.0, 90.0];
% %tripod gait
% phaseijmj = 360*[   0.0  0.5   0.0   0.0   0.0  0.5;
%                    -0.5  0.0  -0.5   0.0   0.0  0.0;
%                     0.0  0.5   0.0   0.5   0.0  0.0;
%                     0.0  0.0  -0.5   0.0  -0.5  0.0;
%                     0.0  0.0   0.0   0.5   0.0  0.5;
%                    -0.5  0.0   0.0   0.0  -0.5  0.0 ];
%        

% phaseij= [180*coupling_strength,0,180*coupling_strength,0,180*coupling_strength,0];
phaseij= [0, 180, 90, 0, 180, 270];
% phaseij= [90, 45, 0, 0, 270, 180];
% sum columns to get phase angle
phaseij= phaseij*(pi/180);


x = [sin(w*t + phaseij(1));
     sin(w*t + phaseij(2));
     sin(w*t + phaseij(3));
     sin(w*t + phaseij(4));
     sin(w*t + phaseij(5));
     sin(w*t + phaseij(6));
     ];
    
y = [cos(w*t + phaseij(1));
     cos(w*t + phaseij(2));
     cos(w*t + phaseij(3));
     cos(w*t + phaseij(4));
     cos(w*t + phaseij(5));
     cos(w*t + phaseij(6));
     ];

y_dot = [alpha*(mu-r^2)*y + w*x;
         alpha*(mu-r^2)*y + w*x;
         alpha*(mu-r^2)*y + w*x;
         alpha*(mu-r^2)*y + w*x;
         alpha*(mu-r^2)*y + w*x;
         alpha*(mu-r^2)*y + w*x;
         ];
     
x_dot = [beta*(mu-r^2)*x - w*y;
         beta*(mu-r^2)*x - w*y;
         beta*(mu-r^2)*x - w*y;
         beta*(mu-r^2)*x - w*y;
         beta*(mu-r^2)*x - w*y;
         beta*(mu-r^2)*x - w*y;
 ];

coupling_strength=1;


th1 = k(1)*y+b(1); 
th2 = zeros(6,s);
th3 = zeros(6,s);


for i = 1:6
    for j = 1:s
        if y_dot(j) >=0
           th2(i,j) = k(2)*x(j)+ b(2);
        else
           th2(i,j) = k(3)*x(j) + b(3);
        end
        
        if y_dot(j) >=0
           th3(i,j) = k(4)*th2(j)+ b(4);
        else
           th3(i,j) = k(5)*th2(j) + b(5);
        end
    end
end

% f = figure;

for i = 1:6
    subplot(2,3,i)
    plot(t,th1(i,:),'-*',t,th2(i,:),'-o',t,th3(i,:),'-x')
    
    legend('Theta '+string(i)+'1' ,'Theta '+string(i)+'2','Theta '+string(i)+'3')
end



figure;
subplot(2,1,1)
plot(t,y_dot(1,:),'-x')
title('Estimatation of calculated y state derivative for leg 1')
subplot(2,1,2)
plot(t,x_dot(1,:),'-x')
ylabel('ampltitude')
xlabel('time /s')
% hold on
% subplot(2,2,1);
% 
% plot(t,x(1,:),t,y(1,:))
% 
% 
% subplot(2,2,2);
% plot(t,y_dot(1,:))
% 
% subplot(2,2,3);
% plot(t,th2(1,:))
% 
% plot(t,x(2,:))
% plot(t,x(3,:))
% plot(t,x(4,:))
% plot(t,x(5,:))
% plot(t,x(6,:))
% ylabel('oscillator')
% xlabel('time /s')
% zlabel('angle in degrees')
% hold off
% mesh(t,z,x)
% colorbar