%In SI
% data rocket 
m = 32.267; %mass after burn
rad = 0.0785; %radius
A_r = 0.019359; %frontal area
Cd = 0.4; %less than 4% error

%data airbrakes 
% remember u goes from 0 to 1
l_f = 0.07; %length of the flap
l_eff = 0.05; %ceffective length of the flap
w_f = 0.07; % width of flap
a_max = asin(l_eff/l_f); %max angle of deployment 
l_act = 0.0508; %linear actuator
t_act = 3.4; %time ofor full deployment at full capacity


%A_f = 4*(w_f*l_eff)*sin(u*a_max)^2; %area per flap is in terms of u 
Cd_f = 1.5; %complete and utter estimation, consider flow and other stuff negligible, and it also linearizes 
%K_u = A_f*Cd_f;


%environment 
rho = 1.161; %density kg/m^3
g = 9.8; % gravity

%initial states
t0 = 4.4; %burnout time
h0 = 746.2; % Altitude after burnout
v0 = 316.84; %
M0 = 0.922; %mach numbrer
v_fs = 317.956; %freestream vel

%model 

% r_d = v0 + v_d;
% v_d = -g -1/(2*m)*pho*v_d^2*K_u;
% x = [r_d; v_d];

tspan = [0 30]; %30 seconds simulation

% Airbrake deployments to test
u_vals = [0, 0.25, 0.5, 0.75, 1];

figure; hold on;
for u = u_vals
    f = @(t, x) [
        x(2);
        -g - (0.5 / m) * rho * x(2)^2 * ...
        (Cd * A_r + Cd_f * 4 * w_f * l_eff * sin(u * a_max)^2)
    ];
    [t, x] = ode45(f, tspan, x0);
    plot(t, x(:,1), 'DisplayName', sprintf('u = %.2f', u));
end

xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude vs. Time for Different Airbrake Deployments');
legend('Location', 'best');
grid on;