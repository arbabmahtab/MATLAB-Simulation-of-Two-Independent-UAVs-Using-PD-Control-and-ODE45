clear; clc; close all;

t_total = 100;
dt = 0.1;
tspan = 0:dt:t_total;

wp = [100 25];

y0 = [0 0 0 0.001 0.001];
tau_ = 0.1;

Kp = 1.5;
Kd = 1.2;

Kp_a = 15.5;
Kd_a = 14.2;

t_all = [];
y_all = [];
u_all = [];

distance_old = 0;
angle_error_old = 0;

for j = 1:length(tspan)-1

    x_target = wp(1);
    y_target = wp(2);

    distance = sqrt((x_target-y0(1))^2 + (y_target-y0(2))^2);
    angle_error = atan2(y_target-y0(2),x_target-y0(1));

    distance_dot = (distance-distance_old)/dt;
    angle_error_dot = (angle_error-angle_error_old)/dt;

    distance_old = distance;
    angle_error_old = angle_error;

    if distance > 1.0
        V_d = Kp*distance + Kd*distance_dot;
        a_d = Kp_a*angle_error + Kd_a*angle_error_dot;

        V_d = max(min(V_d,10),-10);
        a_d = max(min(a_d,20),-20);
    else
        
        break;
    end

    u = [V_d, a_d];
    t_current = [tspan(j), tspan(j+1)];

    [t,y] = ode45(@(t,y) UAV_dynamics(t,y,u,tau_),t_current,y0);

    y0 = y(end,:);
    y_all = [y_all; y(end,:)];
    t_all = [t_all; t(end)];
    u_all = [u_all; u];

end

plot(y_all(:,1),y_all(:,2));
hold on;
plot(wp(1),wp(2),'ro');
xlabel('x-position');
ylabel('y-position');
grid on;

function dydt = UAV_dynamics(t,y,u,tau_)

    V_d = u(1);
    a_d = u(2);

    gamma = y(3);
    V = y(4);
    a = y(5);

    dydt(1) = V*cos(gamma);
    dydt(2) = V*sin(gamma);
    dydt(3) = a/V;
    dydt(4) = (V_d-V)/tau_;
    dydt(5) = (a_d-a)/tau_;

    dydt = dydt.';

end