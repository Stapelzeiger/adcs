clf;
% close all;


inertia = [1, 2, 3];
delta_t = 0.01;
initial_rate = [0.01; 1; 0];
measurement_noise = 0.1; % standard deviation of noise on vector components
perturbation_torque = 1;

kalman_Q = diag([ones(1, 4)*0.001^2, ones(1, 3)*0.001^2]);
kalman_R = eye(6)*measurement_noise^2;

kalman_initial_P = diag([ones(1, 4)*3^2, ones(1, 3)*10^2]);
kalman_initial_att = [cos(pi/4); 0; 0; sin(pi/4)];
kalman_initial_rate =  [0; 0; 0];

sim = Simulation3DBody(delta_t, inertia, initial_rate, measurement_noise, perturbation_torque);
sim.setKalmanCovariancesQR(kalman_Q, kalman_R);
sim.setKalmanInitialState([kalman_initial_att; kalman_initial_rate], kalman_initial_P);

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


hold on
rate_plt_nb_pts = 300;
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


figure(3)
clf
subplot(2,1,1)
attitude_error = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
attitude_error_stddev = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
title('attitude error')
legend('error', 'standard deviation')
ylim([0, 0.5])
subplot(2,1,2)
rate_error = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
rate_error_stddev = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
title('rate error')
legend('error', 'standard deviation')
ylim([0, 1])


figure(4)
clf
P_image = imagesc(sim.kalman.K.P, [-0.001 0.001]);
colormap(jet(100))
colorbar
title('P')

figure(5)
clf
K_image = imagesc(zeros(7, 6), [-1 1]*0.001);
colormap(jet(100))
colorbar
title('K')

figure(6)
clf
eigenv_plot = plot([1],'o');
title('Eigenvalues of F - KH')
xlim([0, 1])
ylim([-1, 1])


redraw_cntdwn = 0;
for t = 0:delta_t:60
    redraw_cntdwn = redraw_cntdwn - delta_t;
    if (redraw_cntdwn <= 0)
        redraw_cntdwn = delta_t * 10;

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

        body_rate = sim.body.getRate();
        kalman_rate = sim.kalman.get_omega();
        addpoints(roll,t,body_rate(1));
        addpoints(roll_estim,t,kalman_rate(1));
        addpoints(pitch,t,body_rate(2));
        addpoints(pitch_estim,t,kalman_rate(2));
        addpoints(yaw,t,body_rate(3));
        addpoints(yaw_estim,t,kalman_rate(3));

        state_var = diag(sim.kalman.K.P);
        rate_err = norm(sim.body.getRate() - sim.kalman.get_omega);
        att_err = quatmult(sim.body.getAttitude, quatconj(sim.kalman.get_attitude));
        att_err = norm(att_err(2:4));
        addpoints(attitude_error,t, att_err);
        addpoints(attitude_error_stddev,t, sqrt(max(state_var(1:4))));
        addpoints(rate_error,t, rate_err);
        addpoints(rate_error_stddev,t, sqrt(max(state_var(5:7))));

        set(P_image,'CData',sim.kalman.K.P)
        set(K_image,'CData',sim.kalman.K.inspect_K)

        e = eig(sim.kalman.K.inspect_F - sim.kalman.K.inspect_K * sim.kalman.K.inspect_H);
        set(eigenv_plot, 'XData', real(e), 'YData', imag(e))

        % pause(delta_t);
        drawnow
    end

    sim.update()
end
