clf;
close all;



inerta = [1, 2, 3];
% inerta = [1.1, 1.9, 3.03];
b = RotationBody3D(diag(inerta));
b.setRate([0.1; 10; 0]);


delta_t = 0.001;
k = Kalman3DBody(delta_t, inerta);


axis equal;
grid on;
xlabel('X','FontSize',18, 'Color','r');
ylabel('Y','FontSize',18, 'Color','g')
zlabel('Z','FontSize',18, 'Color','b')
h = gca; % handle of the figure
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


hold on
rate_plt_nb_pts = 300;
figure
subplot(3,1,1)
roll = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r');
roll_estim = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','r', 'LineStyle', ':');
title('roll (X)')
subplot(3,1,2)
pitch = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','g');
pitch_estim = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','g', 'LineStyle', ':');
title('pitch (Y)')
subplot(3,1,3)
yaw = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b');
yaw_estim = animatedline('MaximumNumPoints',rate_plt_nb_pts, 'Color','b', 'LineStyle', ':');
title('yaw (Z)')


figure
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
ylim([0, 10])


figure
P_image = imagesc(k.K.P, [-0.01 0.01]);
colormap(jet(100))
colorbar

redraw_cntdwn = 0.1;
for t = 0:delta_t:60
    redraw_cntdwn = redraw_cntdwn - delta_t;
    if (redraw_cntdwn <= 0)
        redraw_cntdwn = 0.01;

        omega = b.getRate;
        Lb = b.getInertia * omega;
        L = rotate_by_quaternion(Lb, b.getAttitude);
        ang_mom_v.set('XData', [0, L(1)], 'YData', [0, L(2)], 'ZData', [0, L(3)]);

        omega_i = rotate_by_quaternion(omega, b.getAttitude);
        omega_v.set('XData', [0, omega_i(1)], 'YData', [0, omega_i(2)], 'ZData', [0, omega_i(3)]);

        cube_plot(body_plot,[0,0,0],inerta(1),inerta(2),inerta(3), b.getAttitude);
        cube_plot(estim_plot,[6,0,0],inerta(1),inerta(2),inerta(3), k.get_attitude);

        body_rate = b.getRate();
        kalman_rate = k.get_omega();
        addpoints(roll,t,body_rate(1));
        addpoints(roll_estim,t,kalman_rate(1));
        addpoints(pitch,t,body_rate(2));
        addpoints(pitch_estim,t,kalman_rate(2));
        addpoints(yaw,t,body_rate(3));
        addpoints(yaw_estim,t,kalman_rate(3));

        state_var = diag(k.K.P);
        rate_err = norm(b.getRate() - k.get_omega);
        att_err = quatmult(b.getAttitude, quatconj(k.get_attitude));
        att_err = norm(att_err(2:4));
        addpoints(attitude_error,t, att_err);
        addpoints(attitude_error_stddev,t, sqrt(max(state_var(1:4))));
        addpoints(rate_error,t, rate_err);
        addpoints(rate_error_stddev,t, sqrt(max(state_var(5:7))));

        set(P_image,'CData',k.K.P)
        % pause(delta_t);
        drawnow
    end

    b.update([0; 0; 0], delta_t);
    k.update();
    k.measure(b.measureVector([0; 0; 1], 0.1), ...
              b.measureVector([0; 1; 0], 0.1))
end
