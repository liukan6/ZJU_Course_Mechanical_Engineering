num = [0.25 0];
den = [1 -0.5 -0.5];
sys = tf(num, den, 1); % 假设采样时间为1
step(sys);