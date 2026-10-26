clear; clc; close all;

t_total = 100;
dt = 0.1;
tspan = 0:dt:t_total;

wp = [100 25];


tau_ = 0.1;

Kp = 1.5;
Kd = 1.2;
Kp_a = 15.5;
Kd_a = 14.2;

% Initial states of two UAVs
y01 = [0 0 0 0.001 0.001];
y02 = [-5 -5 0 0.001 0.001];

y_all1 = [];
y_all2 = [];

% UAV 1
y0 = y01; % copy the initial state of UAV 1 into the variable y0
distance_old = 0;
angle_error_old = 0;

for j = 1:length(tspan)-1

    distance = sqrt((wp(1)-y0(1))^2 + (wp(2)-y0(2))^2);
    angle_error = atan2(wp(2)-y0(2),wp(1)-y0(1));

    distance_dot = (distance-distance_old)/dt;
    angle_error_dot = (angle_error-angle_error_old)/dt;

    distance_old = distance;
    angle_error_old = angle_error;

    if distance <= 1.0
        break;
    end

    V_d = Kp*distance + Kd*distance_dot;
    a_d = Kp_a*angle_error + Kd_a*angle_error_dot;

    V_d = max(min(V_d,10),-10);
    a_d = max(min(a_d,20),-20);

    u = [V_d, a_d];

    [t,y] = ode45(@(t,y) UAV_dynamics(t,y,u,tau_), ...
                  [tspan(j),tspan(j+1)],y0);

    y0 = y(end,:);
    y_all1 = [y_all1; y0];

end

% UAV 2: Extra for loop
y0 = y02;
distance_old = 0;
angle_error_old = 0;

for j = 1:length(tspan)-1

    distance = sqrt((wp(1)-y0(1))^2 + (wp(2)-y0(2))^2);
    angle_error = atan2(wp(2)-y0(2),wp(1)-y0(1));

    distance_dot = (distance-distance_old)/dt;
    angle_error_dot = (angle_error-angle_error_old)/dt;

    distance_old = distance;
    angle_error_old = angle_error;

    if distance <= 1.0
        break;
    end

    V_d = Kp*distance + Kd*distance_dot;
    a_d = Kp_a*angle_error + Kd_a*angle_error_dot;

    V_d = max(min(V_d,10),-10);
    a_d = max(min(a_d,20),-20);

    u = [V_d, a_d];

    [t,y] = ode45(@(t,y) UAV_dynamics(t,y,u,tau_), ...
                  [tspan(j),tspan(j+1)],y0);

    y0 = y(end,:);
    y_all2 = [y_all2; y0];

end

% Plot both UAV trajectories
plot(y_all1(:,1),y_all1(:,2),'b','LineWidth',1.5);
hold on;
plot(y_all2(:,1),y_all2(:,2),'r','LineWidth',1.5);
plot(wp(1),wp(2),'ko','MarkerFaceColor','g');

xlabel('x-position');
ylabel('y-position');
legend('UAV 1','UAV 2','Target');
grid on;
axis equal;

function dydt = UAV_dynamics(t,y,u,tau_)

    V_d = u(1);
    a_d = u(2);

    gamma = y(3);
    V = y(4);
    a = y(5);

    dydt = [V*cos(gamma);
            V*sin(gamma);
            a/V;
            (V_d-V)/tau_;
            (a_d-a)/tau_];

end