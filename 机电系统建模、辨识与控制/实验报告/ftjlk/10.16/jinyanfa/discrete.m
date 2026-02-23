G = tf(m*l, [I + m*l^2, 0, -m*g*l]);
Ts = 0.005;
G_d = c2d(G, Ts, 'zoh');
disp(G_d.Numerator);
