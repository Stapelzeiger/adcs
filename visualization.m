clf;
% close all;


% setup simulation
delta_t = 0.1; % [s]
simulation_duration = 60*5; % [s]
speedup = 1;

inertia = [1, 1.1, 1.2];
initial_rate = [0.01; 0.3; 0];
measurement_noise = 4/180*pi; % standard deviation of noise on vector components
perturbation_torque = 0.0001;

rate_gyro_white_noise_deg_p_s = 0.03; % [deg/s/sqrt(Hz)]
rate_gyro_white_noise = rate_gyro_white_noise_deg_p_s/180*pi; % [rad/s/sqrt(Hz)]
rate_gyro_bias_instability_deg_p_s = 0.003; % [deg/s]
rate_gyro_bias_instability_time = 200; % [s]
rate_gyro_bias_random_walk_white_noise = (rate_gyro_bias_instability_deg_p_s/sqrt(rate_gyro_bias_instability_time))/180*pi; % [rad/s/sqrt(Hz)]

filter_model = 'mekf_cst_mom' % one of 'mekf_cst_mom', 'mekf_gyro', 'basic'

sim = Simulation3DBody(filter_model, ...
                       delta_t, ...
                       inertia, ...
                       initial_rate, ...
                       measurement_noise, ...
                       perturbation_torque, ...
                       rate_gyro_white_noise, ...
                       rate_gyro_bias_random_walk_white_noise);



% setup 3D plot
figure(1)
clf
axis equal;
grid on;
xlabel('X','FontSize',18, 'Color','r');
ylabel('Y','FontSize',18, 'Color','g')
zlabel('Z','FontSize',18, 'Color','b')
h = gca;
h.XLim = [-3 9];
h.YLim = [-3 3];
h.ZLim = [-3 3];
h.XTick = -1:1:1;
h.YTick = -1:1:1;
h.ZTick = -1:1:1;
view(30,30);
body_plot = patch('FaceColor', 'flat');
estim_plot = patch('FaceColor', 'flat'); %, 'FaceAlpha',.7
ang_mom_v = patch('EdgeColor', 'c');
omega_v = patch('EdgeColor', 'r');
z1_v = patch('EdgeColor', 'b');
z2_v = patch('EdgeColor', 'b');


% Setup rate plot
hold on
rate_plt_nb_pts = 200;
figure(2)
clf
subplot(3,1,1)
roll = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
roll_estim = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r', 'LineStyle', ':');
title('roll rate')
subplot(3,1,2)
pitch = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','g');
pitch_estim = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','g', 'LineStyle', ':');
title('pitch rate')
subplot(3,1,3)
yaw = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
yaw_estim = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b', 'LineStyle', ':');
title('yaw rate')


% setup error plot
figure(3)
clf
subplot(2,1,1)
attitude_error = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
attitude_error_stddev = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
title('attitude error [deg]')
legend('error', 'standard deviation')
ylim([0, 90])
subplot(2,1,2)
rate_error = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
rate_error_stddev = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
title('rate error [deg/s]')
legend('error', 'standard deviation')
ylim([0, 50])



redraw_prescaler = 1; % update plots every nth iteration
redraw_cntdwn = 0;
for t = 0:delta_t:simulation_duration
    redraw_cntdwn = redraw_cntdwn - delta_t;
    if (redraw_cntdwn <= 0)
        redraw_cntdwn = delta_t * redraw_prescaler;

        % 3D visualization update
        omega = sim.body.getRate;
        Lb = sim.body.getInertia * omega;
        L = rotate_by_quaternion(Lb, sim.body.getAttitude);
        ang_mom_v.set('XData', [0, L(1)], 'YData', [0, L(2)], 'ZData', [0, L(3)]);

        omega_i = rotate_by_quaternion(omega, sim.body.getAttitude);
        omega_v.set('XData', [0, omega_i(1)], 'YData', [0, omega_i(2)], 'ZData', [0, omega_i(3)]);
        z1 = 10*rotate_by_quaternion(sim.inspect_z1, sim.kalman.get_attitude);
        z2 = 10*rotate_by_quaternion(sim.inspect_z2, sim.kalman.get_attitude);
        z1_v.set('XData', [0, z1(1)] + 6, 'YData', [0, z1(2)], 'ZData', [0, z1(3)]);
        z2_v.set('XData', [0, z2(1)] + 6, 'YData', [0, z2(2)], 'ZData', [0, z2(3)]);

        cube_plot(body_plot,[0,0,0],inertia(1),inertia(2),inertia(3), sim.body.getAttitude);
        cube_plot(estim_plot,[6,0,0],inertia(1),inertia(2),inertia(3), sim.kalman.get_attitude);

        % rate plot update
        body_rate = sim.body.getRate();
        kalman_rate = sim.kalman.get_omega();
        addpoints(roll,t,body_rate(1));
        addpoints(roll_estim,t,kalman_rate(1));
        addpoints(pitch,t,body_rate(2));
        addpoints(pitch_estim,t,kalman_rate(2));
        addpoints(yaw,t,body_rate(3));
        addpoints(yaw_estim,t,kalman_rate(3));

        % error plot update
        state_var = diag(sim.kalman.K.P);
        rate_err = norm(sim.body.getRate() - sim.kalman.get_omega);
        rate_err_deg_per_sec = rate_err * 180 / pi;
        att_err = quatmult(sim.body.getAttitude, quatconj(sim.kalman.get_attitude));
        att_err = asin(norm(att_err(2:4)))*2;
        att_err_deg = att_err * 180 / pi;
        addpoints(attitude_error,t, att_err_deg);
        att_err_stddev = sqrt(sum(state_var(1:3)));
        att_err_stddev_deg = att_err_stddev * 180 / pi;
        addpoints(attitude_error_stddev,t, att_err_stddev_deg);
        addpoints(rate_error,t, rate_err_deg_per_sec);
        rate_err_stddev = sqrt(sum(state_var(4:6)));
        rate_err_stddev_deg_per_sec = rate_err_stddev * 180 / pi;
        addpoints(rate_error_stddev,t, rate_err_stddev_deg_per_sec);

        drawnow
    end
    if (speedup == 0)
        pause(delta_t);
    end
    sim.update()
end
