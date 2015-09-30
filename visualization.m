clf;
figure(1);
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

hold off;

body_plot = patch('FaceColor', 'flat');
estim_plot = patch('FaceColor', 'flat', 'FaceAlpha',.7);
ang_mom_v = patch('EdgeColor', 'c');
omega_v = patch('EdgeColor', 'r');

inerta = [1, 2, 3];
b = RotationBody3D(diag(inerta));
% e = RotationBody3D(diag(inerta));
b.setRate([0.1; 10; 0]);
% e.setRate([0.2; 10; 0]);


delta_t = 0.01;
k = Kalman3DBody(delta_t, inerta(1), inerta(2), inerta(3));
for t = 0:delta_t:60
    b.update([0; 0; 0], delta_t);
    k.update();
    k.measure(b.measureVector([1; 0; 0], 0.01), b.measureVector([0; 1; 0], 0.01))

    omega = b.getRate;
    Lb = b.getInertia * omega;
    L = rotate_by_quaternion(Lb, b.getAttitude);
    ang_mom_v.set('XData', [0, L(1)], 'YData', [0, L(2)], 'ZData', [0, L(3)]);

    omega_i = rotate_by_quaternion(omega, b.getAttitude);
    omega_v.set('XData', [0, omega_i(1)], 'YData', [0, omega_i(2)], 'ZData', [0, omega_i(3)]);

    % e.update([0; 0; 0], delta_t);
    cube_plot(body_plot,[0,0,0],inerta(1),inerta(2),inerta(3), b.getAttitude);
    cube_plot(estim_plot,[6,0,0],inerta(1),inerta(2),inerta(3), k.get_attitude);
    pause(delta_t);
end
