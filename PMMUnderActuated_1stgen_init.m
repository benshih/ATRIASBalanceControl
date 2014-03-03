%% Simulation Parameters

clc
clear
close all

t_final = 1;

% Enable closed loop feedback
on = 0;

g = 9.81;
I_m = 7;
I_t = 1.4;
tau_max = 5000; % 5000 or inf

kp_leg = 1000;
kd_leg = kp_leg/10;
l_l2 = 1;

kp_torso = 1000;
kd_torso = kp_torso/10;
l_torso = 0.8;
m_torso = 44;

% Initial angles of robot [deg]
% Note: bad initial angles can result in singularities in the revolute
% joints.
torso_leg2_angle_init = 270;
torso_leg1_angle_init = 180;
l_l1_init = 1;
alpha_angle_init = 90;

% Desired orientation of robot
xt_des = 0;
yt_des = 1.8;
th_des_deg = 0;



%% Simulation Data

simOpts = simset('srcworkspace','current','returnworkspaceoutputs','on');
simout = sim('PMMUnderActuated_1stgen', t_final, simOpts);

% Simulation time steps
t = simout.get('tout');

% Torso accelerations
xtdd_des = simout.get('xtdd_des');
ytdd_des = simout.get('ytdd_des');
thdd_des = simout.get('thdd_des');
dat = simout.get('posdd_meas');
xtdd_meas = dat(:, 1);
ytdd_meas = dat(:, 2);
thdd_meas = simout.get('thdd_meas');

% Commanded torques/forces
tau_1 = simout.get('tau_1');
tau_2 = simout.get('tau_2');
f_leg = simout.get('F_l');

% Computed torques/forces
tau_1_sat = simout.get('tau_1_sat');
tau_2_sat = simout.get('tau_2_sat');
f_leg_sat = simout.get('f_leg_sat');

% Measured joint angles
p1 = simout.get('p1_meas');
p2 = simout.get('p2_meas');

% Body sensor measurements
l_l1 = simout.get('l_meas');
xt = simout.get('xt_meas');
yt = simout.get('yt_meas');

% Lagrangian Model values
% x = [GRFx GRFy T1 T2 Fl p1dd p2dd ldd] == things I can sense and plot
% below
% Things I'm commanding with simple control at the moment - xtdd ytdd thdd
dat = simout.get('model_calc');
GRFx_calc = dat(:, 1);
GRFy_calc = dat(:, 2);
p1dd_calc = dat(:, 6);
p2dd_calc = dat(:, 7);
ldd_calc = dat(:, 8);
dat = simout.get('a_ad_add_grfx_grfy_grfz');
a = dat(:, 1);
ad = dat(:, 2);
add = dat(:, 3);
GRFx_meas = dat(:, 4);
GRFy_meas = dat(:, 5);
p1dd_meas = simout.get('p1dd_meas');
p2dd_meas = simout.get('p2dd_meas');
ldd_meas = simout.get('ldd_meas');
th = a + pi/2 - p1;


% Convert radians to degrees
p1_deg = 180/pi*p1;
p2_deg = 180/pi*p2;
th_deg = 180/pi*th;

%% Signal Visualization

close all

i = 1;
% measured xtdd vs desired control value
figure(i)
i = i+1;
hold on;
plot(t, xtdd_meas, 'r');
plot(t, xtdd_des, 'b');
title('xdd_{torso}');
legend('measured', 'desired');
hold off;

% measured ytdd vs desired control value
figure(i)
i = i+1;
hold on;
plot(t, ytdd_meas, 'r');
plot(t, ytdd_des, 'b');
title('ydd_{torso}');
legend('measured', 'desired');
hold off;

% measured thdd vs desired control value
figure(i)
i = i+1;
hold on;
plot(t, thdd_meas, 'r');
plot(t, thdd_des, 'b');
title('thdd');
legend('measured', 'desired');
hold off;

% measured ldd vs lagrangian model
figure(i)
i = i+1;
hold on;
plot(t, ldd_meas, 'r');
plot(t, ldd_calc, 'b');
title('ldd');
legend('measured', 'calculated');
hold off;

% measured p1dd vs lagrangian model
figure(i)
i = i+1;
hold on;
plot(t, p1dd_meas, 'r');
plot(t, p1dd_calc, 'b');
title('p1dd');
legend('measured', 'calculated');
hold off;

% measured p2dd vs lagrangian model
figure(i)
i = i+1;
hold on;
plot(t, p2dd_meas, 'r');
plot(t, p2dd_calc, 'b');
title('p2dd');
legend('measured', 'calculated');
hold off;

% measured GRFx vs lagrangian model
figure(i)
i = i+1;
hold on;
plot(t, GRFx_meas, 'r');
plot(t, GRFx_calc, 'b');
title('GRFx');
legend('measured', 'calculated');
hold off;

% measured GRFy vs lagrangian model
figure(i)
i = i+1;
hold on;
plot(t, GRFy_meas, 'r');
plot(t, GRFy_calc, 'b');
title('GRFy');
legend('measured', 'calculated');
hold off;

% computed torques/forces
figure(i)
i = i+1;
hold on;
plot(t, tau_1, 'r');
plot(t, tau_2, 'b');
plot(t, f_leg, 'g');
title('computed actuation');
legend('tau1', 'tau2', 'fleg');
hold off;

% commanded torques/forces
figure(i)
i = i+1;
hold on;
plot(t, tau_1_sat, 'r');
plot(t, tau_2_sat, 'b');
plot(t, f_leg_sat, 'g');
title('commanded actuation');
legend('tau1_{sat}', 'tau2_{sat}', 'fleg_{sat}');
hold off;

% Plot joint angles
figure(i)
i = i+1;
hold on;
plot(t, p1_deg, 'r');
plot(t, p2_deg, 'b');
plot(t, th_deg, 'g');
title('joint angles');
legend('p1', 'p2', 'th');
hold off;

%% Robot Animation

figure;
pause on
grid on
a = 8;

g1 = th + p1;
g2 = th + p2;

xh = -l_l1.*cos(a);
yh = l_l1.*sin(a);
xt = xh + l_torso.*sin(th);
yt = yh + l_torso.*cos(th);
xf2 = xh + l_l2.*sin(g2);
yf2 = yh + l_l2.*cos(g2);
xf = xh + l_l1.*sin(g1);
yf = yh + l_l1.*cos(g1);

for i = 1:length(t)
    clf;
    grid on 
    axis([-a a -a a]);
    xtp = xt(i);
    ytp = yt(i);
    thp = th(i);
    xfp = xf(i);
    yfp = yf(i);
    xhp = xh(i);
    yhp = yh(i);
    xf2p = xf2(i);
    yf2p = yf2(i);
    
    
    line('XData', [xtp xhp], 'YData', [ytp yhp], 'Color', [1 0 0], 'LineWidth', 3); % torso red
    line('XData', [xhp xfp], 'YData', [yhp yfp], 'Color', [0 1 0], 'LineWidth', 3); % leg1 green
    line('XData', [xhp xf2p], 'YData', [yhp yf2p], 'Color', [0 0 1], 'LineWidth', 3); % leg2 blue
    
    legend('torso', 'leg1', 'leg2');

    pause(0.1)
end

length_torso = sqrt((xt-xh).^2 + (yt-yh).^2);
length_leg1 = sqrt((xh-xf).^2 + (yh-yf).^2);
length_leg2 = sqrt((xh-xf2).^2 + (yh-yf2).^2);

figure;
hold on
plot(t, length_torso, 'r');
plot(t, length_leg1, 'b');
plot(t, length_leg2, 'g');
legend('torso length', 'leg1 length', 'leg2 length');
hold off