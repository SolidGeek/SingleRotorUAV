blue        = '#0051ff';
light_blue  = '#b3cbff';
red         = '#ff0000';
light_red   = '#ffb3b3';
green       = '#06ad00';
light_green = '#9de69a';
black       = '#000000';


data = out.attitude;
time = data.Time;
roll = data.Data(:,1);

t = time( time > 19.5 & time < 21) - 19.5;
roll = roll( time > 19.5 & time < 21);

step = [0, 0, 0.1, 0.1];
step_t = [0, 0.5, 0.5, 1.5];

figure(1)
hold on
plot(t,roll, 'LineWidth', 2, 'Color', red);
stairs(step_t, step, '--', 'Color', black);
grid on
xlabel("Time [s]");
xticks(0:.25:1.5);
ylabel("Amplitude [rad]")
legend(["Roll", "Step"], 'Location', 'southeast');

set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 13)
set(gcf, 'Position', [100, 100, 500, 400])
hold off


data = out.altitude;
time = data.Time;
alt = data.Data;

t = time( time > 0 & time < 10);
alt = alt( time > 0 & time < 10);

step = [0, 0, 1, 1];
step_t = [0, 1, 1, 10];

figure(2)
hold on
plot(t, alt, 'LineWidth', 2, 'Color', green)
stairs(step_t, step, '--', 'Color', black);
grid on
xlabel("Time [s]");
xticks(0:1:10);
ylabel("Amplitude [m]")
legend(["Altitude", "Step"], 'Location', 'southeast');

set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 13)
set(gcf, 'Position', [100, 100, 500, 400])
hold off
