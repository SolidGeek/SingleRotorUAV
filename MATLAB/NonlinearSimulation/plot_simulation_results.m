close all

blue        = '#0051ff';
light_blue  = '#b3cbff';
red         = '#ff0000';
light_red   = '#ffb3b3';
green       = '#06ad00';
light_green = '#9de69a';
black       = '#000000';

% Position 
pos = out.pos;
t = pos.Time;

x = pos.Data(:,1);
y = pos.Data(:,2);
z = pos.Data(:,3);


%% Position Step

figure

tiledlayout(3,1)
ax1 = nexttile;
plot( t, x, 'LineWidth', 2, 'Color', red );
grid on
xlabel("Time [s]")
ylabel("X-position [m]")

ax2 = nexttile;
plot( t, y, 'LineWidth', 2, 'Color', blue );
grid on
xlabel("Time [s]")
ylabel("Y-position [m]")

ax3 = nexttile;
plot( t, z, 'LineWidth', 2, 'Color', green );
grid on
xlabel("Time [s]")
ylabel("Z-position [m]")

linkaxes([ax1 ax2 ax3],'xy')