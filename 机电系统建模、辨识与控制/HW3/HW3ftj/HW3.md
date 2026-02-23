## A. Requirement of controller structure

### Requirements

(i) Zero steady-state error for constant friction torque of rotary arm.

(ii) 2% settling time of the closed-loop system should be less than 0.5sec. 

(iii) OS% should be no more than 20%

### Analyze

For (ii):

$t_s(\pm2\%)=\frac4{\xi\omega_n}=\frac4{\sigma}\leqslant0.5$

$\therefore\sigma\geqslant8$



For (iii):
$OS\%=100\%\times e^{-\frac{\zeta\pi}{\sqrt{1-\zeta^2}}}\leqslant20\%$

$\zeta\geqslant\zeta_{min}=\sqrt{\frac1{{\frac\pi {ln(5)}}^2 + 1}}\approx0.4559459$

$\varphi\leqslant \arccos{\zeta_{min}}\approx62.87\degree$



![Aufgabe_A](MATLAB_Ergebnis\Aufgabe_A.png)

<div style="page-break-after: always;"></div>

## B. Proper PID

$C(s)=K_p + \dfrac{K_i}{s} + \dfrac{K_d s}{\tau s+1}$

Placing poles at  [ -10+10j, -10-10j, -50, -50], so the expected polynomial is $s^4 + 120 s^3 + 4700 s^2 + 70000 s + 500000$



### Design results

$\tau_D = 0.0111362, K_P = 16.564, K_I = 137.031, K_D = 0.284031$

$C(s)=\frac{0.4685 s^2 + 18.09 s + 137}{0.01114 s^2 + s}$

$T(s)=\frac{2095 s^2 + 8.091\times10^4 s + 6.129\times10^5}{s^4 + 120 s^3 + 4700 s^2 + 7\times10^4 s + 5\times10^5}$



### Verification by Simulink

![Aufgabe_B](MATLAB_Ergebnis\Aufgabe_B.png)

Performance requirements are satisfied.

<div style="page-break-after: always;"></div>

## C. General Controller

### Cases for controller design with pole placement method

| **Case** |          **Closed-loop bandwidth**          | **Initial rod angle** |
| :------: | :-----------------------------------------: | :-------------------: |
|  Case 1  | ω<sub>cl</sub> = 5, ζ<sub>cl</sub> = 0.707  |        α₀ = 6°        |
|  Case 2  | ω<sub>cl</sub> = 15, ζ<sub>cl</sub> = 0.707 |        α₀ = 6°        |
|  Case 3  | ω<sub>cl</sub> = 25, ζ<sub>cl</sub> = 0.707 |        α₀ = 6°        |
|  Case 4  | ω<sub>cl</sub> = 5, ζ<sub>cl</sub> = 0.707  |       α₀ = 10°        |
|  Case 5  | ω<sub>cl</sub> = 15, ζ<sub>cl</sub> = 0.707 |       α₀ = 10°        |
|  Case 6  | ω<sub>cl</sub> = 25, ζ<sub>cl</sub> = 0.707 |       α₀ = 10°        |

### Verification by Simulink

![case1](MATLAB_Ergebnis\Aufgabe_C\case1.png)

![case1](MATLAB_Ergebnis\Aufgabe_C\case2.png)

![case1](MATLAB_Ergebnis\Aufgabe_C\case3.png)

![case1](MATLAB_Ergebnis\Aufgabe_C\case4.png)

![case1](MATLAB_Ergebnis\Aufgabe_C\case5.png)

![case1](MATLAB_Ergebnis\Aufgabe_C\case6.png)

### Comparison

![alpha_compare](MATLAB_Ergebnis\Aufgabe_C\alpha_compare.png)

![alpha_compare](MATLAB_Ergebnis\Aufgabe_C\theta_compare.png)

### Result

The closed-loop bandwidth determines transient speed: larger $\omega_{cl}$ produces faster rise and settling times, but also larger control effort and reduced robustness to nonlinearity and actuator limits. In simulations the 25-rad/s designs show the smallest settling times but also the largest control peaks.

When the initial pole angle is larger (10°), larger overshoot and oscillation occur compared to the 6°.
