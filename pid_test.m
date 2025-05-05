%% Constants
m = 32.267;
A_r = 0.019359;
Cd = 0.5;

l_f = 0.07;
l_eff = 0.05;
w_f = 0.07;
a_max = asin(l_eff / l_f);
Cd_f = 1.5;

rho = 1.161;
g = 9.8;

%% Operating point (equilibrium)
x_eq = [746.2; 316.84];  % [altitude; velocity]
u_eq = 0;

%% Symbolic Linearization
syms x1 x2 u real

Af = 4*w_f*l_eff*sin(u*a_max)^2;
D = Cd*A_r + Cd_f*Af;

f1 = x2;
f2 = -g - (0.5 / m)*rho*x2^2*D;
f = [f1; f2];

x = [x1; x2];

A_sym = jacobian(f, x);
B_sym = jacobian(f, u);

A_lin = double(subs(A_sym, {x1, x2, u}, {x_eq(1), x_eq(2), u_eq}));
B_lin = double(subs(B_sym, {x1, x2, u}, {x_eq(1), x_eq(2), u_eq}));


%% Discretization
Ts = 0.1;  % sample time
sys_c = ss(A_lin, B_lin, eye(2), zeros(2,1));
sys_d = c2d(sys_c, Ts);
Ad = sys_d.A;
Bd = sys_d.B;

% %% LQR Controller
% Q = diag([10, 1]);  % altitude error is more important
% R = 0.01;
% K = dlqr(Ad, Bd, Q, R);
% 
% %% Closed-Loop Simulation
% x = x_eq';             % start at equilibrium
% x_ref = [750; 0];      % target: hover at 750m, 0 m/s
% 
% T_total = 10;
% N = T_total / Ts;
% 
% X = zeros(2, N);
% U = zeros(1, N);
% 
% for k = 1:N
%     u = -K * (x - x_ref);
%     u = max(0, min(1, u));  % Clamp u to [0, 1]
%     U(k) = u;
% 
%     x = Ad * x + Bd * u;
%     X(:,k) = x;
% end
% 
% time = (0:N-1)*Ts;
% 
% %% Plot Results
% figure;
% subplot(2,1,1);
% plot(time, X(1,:), 'b', 'LineWidth', 1.5); 
% ylabel('Altitude (m)');
% title('Rocket Altitude (LQR Control)');
% grid on;
% 
% subplot(2,1,2);
% plot(time, U, 'r', 'LineWidth', 1.5); 
% ylabel('Airbrake Input u');
% xlabel('Time (s)');
% grid on;
