a = 0;
b = 5;

phi = -38;
xdot = 2;
ydot = -0.5;

x = xdot*cos(deg2rad(phi)) - ydot*sin(deg2rad(phi));
y = xdot*sin(deg2rad(phi)) + ydot*cos(deg2rad(phi));

disp(phi)
disp(x)
disp(y)