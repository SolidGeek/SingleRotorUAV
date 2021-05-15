close all

blue        = '#0051ff';
light_blue  = '#b3cbff';
red         = '#ff0000';
light_red   = '#ffb3b3';
green       = '#06ad00';
light_green = '#9de69a';
black       = '#000000';

% Position 
% 
pos = out.pos;
time = pos.Time;
ref = out.posref;

x = pos.Data(:,1);
y = pos.Data(:,2);
z = pos.Data(:,3);

%% Position
xr = ref.Data(:,1);
yr = ref.Data(:,2);
zr = ones( length(xr) );
zr(1) = 0;

figure(1)
fig1=figure(1);
hold on

% plot3( x_no_yaw, y_no_yaw, z_no_yaw, 'LineWidth', 2, 'Color', blue );
plot( x, y, 'LineWidth', 2, 'Color', red );
plot( xr, yr, '--', 'LineWidth', 1,  'Color', black );

xlim([-1.2 1.2]);
ylim([-1.2 1.2]);
zlim([0 1.2]);
xticks(-1.5:.5:1.5);
yticks(-1.5:.5:1.5);
grid on
axis square
xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")
hold off
% view([pi -pi pi]);
% legend(["Constant yaw", "With yaw control", "Trajectory"], 'Location', 'southeast');
legend(["Position", "Trajectory"], 'Location', 'southeast');
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 13)
set(gcf, 'Position', [100, 100, 500, 500])

% exportgraphics(gcf,'test.pdf','ContentType','vector')


%% Position Step

t = time( time > 18 & time < 34) - 18;
x = x( time > 18 & time < 34);

step = [0, 0, 1, 1];
step_t = [0, 2, 2, 16 ];

figure(2)
hold on
% plot( t_no_anti, x_no_anti, 'LineWidth', 2, 'Color', light_red );
plot( t, x, 'LineWidth', 2, 'Color', red );
stairs(step_t, step, '--', 'Color', black);

xticks(0:2:16);

grid on
axis tight
ylim([-0.2 1.2]);
yticks(-0.2:.2:1.2);
xlabel("Time [s]")
ylabel("Distance [m]")
legend(["Position", "Step"], 'Location', 'southeast');
hold off
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 13)
set(gcf, 'Position', [100, 100, 600, 500])


% data = out.actuation;
% time = data.Time;
% a1 = data.Data(:,1);
% a2 = data.Data(:,2);
% 
% t = time( time > 19.5 & time < 22) - 19.5;
% a1 = a1( time > 19.5 & time < 22);
% a2 = a2( time > 19.5 & time < 22);
% 
% figure(1)
% hold on
% plot(t,a1, 'LineWidth', 2, 'Color', black);
% plot(t,a2, '--', 'LineWidth', 2, 'Color', black);
% stairs(step_t, step, '--', 'Color', black);
% grid on
% xlabel("Time [s]");
% xticks(0:.5:2.5);
% yticks(-6:2:6);
% ylim([-6 6])
% ylabel("Actuation [deg]")
% legend(["\alpha_{1}, \alpha_{3}","\alpha_{2}, \alpha_{4}"], 'Location', 'northeast');
% 
% set(gca, 'FontName', 'Times New Roman')
% set(gca, 'FontSize', 13)
% set(gcf, 'Position', [100, 100, 500, 400])
% hold off
% 
% 
% 
% data = out.attitude;
% time = data.Time;
% roll = data.Data(:,3);
% 
% t = time( time > 19 & time < 24) - 19;
% roll = roll( time > 19 & time < 24);
% 
% step = [0, 0, 1, 1];
% step_t = [0, 1, 1, 5];
% 
% figure(2)
% hold on
% plot(t,roll, 'LineWidth', 2, 'Color', green);
% stairs(step_t, step, '--', 'Color', black);
% grid on
% xlabel("Time [s]");
% xticks(0:1:5);
% ylim([-0.2 1.2]);
% ylabel("Amplitude [rad]")
% legend(["Yaw", "Step"], 'Location', 'southeast');
% 
% set(gca, 'FontName', 'Times New Roman')
% set(gca, 'FontSize', 13)
% set(gcf, 'Position', [100, 100, 500, 400])
% hold off
% 
% 
% data = out.altitude;
% time = data.Time;
% alt = data.Data;


%% Altitude 
% t = time( time > 0 & time < 20);
% alt = z( time > 0 & time < 20);
% 
% step = [0, 0, 1, 1, 2, 2];
% step_t = [0, 1, 1, 10, 10, 20];
% 
% figure(2)
% hold on
% plot(t_no_int, alt_no_int, 'LineWidth', 2, 'Color', light_green)
% plot(t, alt, 'LineWidth', 2, 'Color', green)
% stairs(step_t, step, '--', 'Color', black);
% grid on
% xlabel("Time [s]");
% xticks(0:2:20);
% ylabel("Amplitude [m]")
% legend(["Altitude", "With anti-windup", "Step"], 'Location', 'southeast');
% 
% set(gca, 'FontName', 'Times New Roman')
% set(gca, 'FontSize', 13)
% set(gcf, 'Position', [100, 100, 600, 500])
% hold off
