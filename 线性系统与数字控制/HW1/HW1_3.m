num = [1];
den = [0.02 0.3 1 0];
sys = tf(num, den);
rlocus(sys);