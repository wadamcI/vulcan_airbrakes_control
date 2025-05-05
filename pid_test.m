%% Constants
m = 32.267;
A_r = 0.019359; Cd = 0.5;
l_f = 0.07; l_eff = 0.05; w_f = 0.07;
a_max = asin(l_eff / l_f);
Cd_f = 1.5;
rho = 1.161; g = 9.8;

%% PID gains (tune these)
Kp = 0.005; Ki = 0.0002; Kd = 0.02;

%% Target
h_target = 2500;

%% Simulation setup
x = [746.2; 316.84]; % initial [altitude; velocity]
tspan = 0:0.05:30;
dt = 0.05;

X = zeros(length(tspan), 2);
U = zeros(length(tspan), 1);
I_err = 0;
prev_err = 0;

for k = 1:length(tspan)
    % Error terms
    err = h_target - x(1);
    d_err = (err - prev_err) / dt;
    I_err = I_err + err * dt;
    prev_err = err;

    % PID controller
    u = Kp * err + Ki * I_err + Kd * d_err;
    u = max(0, min(1, u)); % Saturate u

    % Store
    U(k) = u;

    % Nonlinear dynamics
    A_f = 4 * w_f * l_eff * sin(u * a_max)^2;
    D = Cd * A_r + Cd_f * A_f;
    v_dot = -g - (0.5 / m) * rho * x(2)^2 * D;

    % Integrate (Euler)
    x(1) = x(1) + x(2) * dt;
    x(2) = x(2) + v_dot * dt;

    X(k,:) = x;
end

%% Plot
figure;
subplot(2,1,1);
plot(tspan, X(:,1), 'b'); ylabel('Altitude (m)');
title('PID Control on Nonlinear Rocket System'); grid on;

subplot(2,1,2);
plot(tspan, U, 'r'); ylabel('Airbrake u'); xlabel('Time (s)'); grid on;
