### 问题 5：连续时间超前控制器的离散化

给定连续时间超前控制器：
\[ C(s) = \frac{a s + 1}{s + 1} \]

#### (a) 使用 Tustin 近似（双线性变换）求离散等效 \( C(z) \)（10%）

**Tustin 近似公式：**
\[ s = \frac{2}{T} \cdot \frac{z - 1}{z + 1} \]
假设采样时间 \( T \) 未给出，通常可以保留 \( T \) 或假设 \( T = 1 \)（题目未说明，这里保留 \( T \)）。

将 \( s \) 替换为 Tustin 变换：
\[ C(z) = \frac{a \left( \frac{2}{T} \cdot \frac{z - 1}{z + 1} \right) + 1}{\frac{2}{T} \cdot \frac{z - 1}{z + 1} + 1} \]
\[ = \frac{\frac{2a}{T} (z - 1) + (z + 1)}{\frac{2}{T} (z - 1) + (z + 1)} \cdot \frac{z + 1}{z + 1} \]
\[ = \frac{\left( \frac{2a}{T} + 1 \right) z + \left( -\frac{2a}{T} + 1 \right)}{\left( \frac{2}{T} + 1 \right) z + \left( -\frac{2}{T} + 1 \right)} \]
\[ = \frac{(2a + T) z + (-2a + T)}{(2 + T) z + (-2 + T)} \]

**最终结果：**
\[ C(z) = \frac{(2a + T) z + (T - 2a)}{(2 + T) z + (T - 2)} \]

---

#### (b) 使用阶跃不变法求离散等效 \( C(z) \)（5%）

**阶跃不变法步骤：**
1. 计算 \( C(s) \) 的阶跃响应：
   \[ \frac{C(s)}{s} = \frac{a s + 1}{s (s + 1)} = \frac{A}{s} + \frac{B}{s + 1} \]
   部分分式分解：
   \[ A = \lim_{s \to 0} \frac{a s + 1}{s + 1} = 1 \]
   \[ B = \lim_{s \to -1} \frac{a s + 1}{s} = -a + 1 \]
   因此：
   \[ \frac{C(s)}{s} = \frac{1}{s} + \frac{1 - a}{s + 1} \]
   阶跃响应：
   \[ \mathcal{L}^{-1} \left[ \frac{C(s)}{s} \right] = 1 + (1 - a) e^{-t} \]

2. 对阶跃响应采样（\( t = n T \)）：
   \[ y(n T) = 1 + (1 - a) e^{-n T} \]

3. 取 \( z \) 变换：
   \[ Y(z) = \frac{1}{1 - z^{-1}} + (1 - a) \frac{1}{1 - e^{-T} z^{-1}} \]
   \[ = \frac{z}{z - 1} + \frac{1 - a}{1 - e^{-T} z^{-1}} \]

4. 计算 \( C(z) \)：
   \[ C(z) = (1 - z^{-1}) Y(z) \]
   \[ = (1 - z^{-1}) \left( \frac{z}{z - 1} + \frac{1 - a}{1 - e^{-T} z^{-1}} \right) \]
   \[ = 1 + (1 - a) \frac{1 - z^{-1}}{1 - e^{-T} z^{-1}} \]
   \[ = \frac{(1 - e^{-T} z^{-1}) + (1 - a)(1 - z^{-1})}{1 - e^{-T} z^{-1}} \]
   \[ = \frac{1 - e^{-T} z^{-1} + 1 - a - z^{-1} + a z^{-1}}{1 - e^{-T} z^{-1}} \]
   \[ = \frac{2 - a - (e^{-T} + 1 - a) z^{-1}}{1 - e^{-T} z^{-1}} \]
   转换为 \( z \) 的正幂形式：
   \[ C(z) = \frac{(2 - a) z - (e^{-T} + 1 - a)}{z - e^{-T}} \]

**最终结果：**
\[ C(z) = \frac{(2 - a) z - (e^{-T} + 1 - a)}{z - e^{-T}} \]

---

### 问题 6：s 平面区域到 z 平面的映射（\( T = 0.1 \, \text{sec} \)）

#### (a) 矩形区域 \( \sigma \in [-5, -20] \), \( \omega \in [-6, 6] \) 的 z 平面映射（10%）

**映射关系：**
\[ z = e^{s T} = e^{(\sigma + j \omega) T} \]
对于 \( T = 0.1 \)：
\[ z = e^{0.1 \sigma} e^{j 0.1 \omega} \]

**边界分析：**
1. \( \sigma = -5 \)：
   \[ |z| = e^{-0.5} \approx 0.6065 \]
   \( \omega \in [-6, 6] \)：
   \[ \angle z \in [-0.6, 0.6] \, \text{rad} \approx [-34.38^\circ, 34.38^\circ] \]

2. \( \sigma = -20 \)：
   \[ |z| = e^{-2} \approx 0.1353 \]
   \( \omega \in [-6, 6] \)：
   \[ \angle z \in [-0.6, 0.6] \, \text{rad} \]

**MATLAB 代码示例：**
```matlab
T = 0.1;
sigma = linspace(-5, -20, 100);
omega = linspace(-6, 6, 100);
[sigma_grid, omega_grid] = meshgrid(sigma, omega);
z = exp(T * (sigma_grid + 1i * omega_grid));
plot(real(z(:)), imag(z(:)), '.');
axis equal;
xlabel('Re(z)'); ylabel('Im(z)');
title('z-plane mapping of rectangular region');
```

**结果描述：**
- z 平面中对应区域为扇形，幅度范围 \( |z| \in [e^{-2}, e^{-0.5}] \)，角度范围 \( \angle z \in [-0.6, 0.6] \) 弧度。

---

#### (b) 扇形区域 \( \zeta \in [0.5, 0.9] \), \( \omega_n \in [0, 20] \) 的 z 平面映射（10%）

**极坐标表示：**
\[ s = -\zeta \omega_n \pm j \omega_n \sqrt{1 - \zeta^2} \]
对于 \( \zeta \in [0.5, 0.9] \) 和 \( \omega_n \in [0, 20] \)：
- \( \sigma = -\zeta \omega_n \in [-18, 0] \)（因为 \( \omega_n \leq 20 \)）
- \( \omega = \pm \omega_n \sqrt{1 - \zeta^2} \in [-20, 20] \)

**映射到 z 平面：**
\[ z = e^{s T} = e^{-\zeta \omega_n T} e^{\pm j \omega_n T \sqrt{1 - \zeta^2}} \]
对于 \( T = 0.1 \)：
- 幅度 \( |z| = e^{-0.1 \zeta \omega_n} \in [e^{-1.8}, 1] \)
- 角度 \( \angle z = \pm 0.1 \omega_n \sqrt{1 - \zeta^2} \in [-2, 2] \) 弧度（因为 \( \omega_n \leq 20 \)）

**MATLAB 代码示例：**
```matlab
T = 0.1;
zeta = linspace(0.5, 0.9, 100);
omega_n = linspace(0, 20, 100);
[zeta_grid, omega_n_grid] = meshgrid(zeta, omega_n);
sigma = -zeta_grid .* omega_n_grid;
omega = omega_n_grid .* sqrt(1 - zeta_grid.^2);
z1 = exp(T * (sigma + 1i * omega));
z2 = exp(T * (sigma - 1i * omega));
plot(real(z1(:)), imag(z1(:)), '.', real(z2(:)), imag(z2(:)), '.');
axis equal;
xlabel('Re(z)'); ylabel('Im(z)');
title('z-plane mapping of pizza-slice region');
```

**结果描述：**
- z 平面中对应区域为从原点向外发散的扇形，幅度 \( |z| \in [e^{-1.8}, 1] \)，角度随 \( \omega_n \) 和 \( \zeta \) 变化。

---

### 最终答案

#### 问题 5：
(a) Tustin 近似：
\[ C(z) = \frac{(2a + T) z + (T - 2a)}{(2 + T) z + (T - 2)} \]

(b) 阶跃不变法：
\[ C(z) = \frac{(2 - a) z - (e^{-T} + 1 - a)}{z - e^{-T}} \]

#### 问题 6：
(a) 矩形区域映射到 z 平面为扇形，\( |z| \in [e^{-2}, e^{-0.5}] \)，\( \angle z \in [-0.6, 0.6] \) 弧度。  
(b) 扇形区域映射到 z 平面为从原点发散的扇形，\( |z| \in [e^{-1.8}, 1] \)，角度范围 \( \angle z \in [-2, 2] \) 弧度。