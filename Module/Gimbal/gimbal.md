# 二轴云台指向控制
参考：https://zhuanlan.zhihu.com/p/1988696935104811632
## 末端姿态角到关节角的求解
定义世界坐标系，底盘坐标系，云台坐标系，末端姿态坐标系

关节坐标系1：yaw轴为z轴，原点在yaw轴上、pitch轴的高度，x轴与底盘坐标系平行（faiz=0时与底盘系重合）
关节坐标系2：pitch轴为z轴（沿底盘系 -Y 方向，绕z轴正转为抬头），x轴沿连杆2（枪管方向），原点在pitch轴上
连杆1：yaw轴与pitch轴的公垂线，沿x1，长度 l（pitch轴在yaw轴后方，x1取向前，故a1=-l）
连杆2：pitch轴到末端（枪口），沿x2，长度 L（z2之后没有下一条关节轴，取固定偏置）

云台IMU测得云台到世界坐标系的旋转矩阵
底盘IMU测得底盘到世界坐标系的旋转矩阵

定义关节坐标系1关节角为faiz，绕关节坐标系（底盘坐标系）的z轴旋转
定义关节坐标系2关节角为theta，绕关节坐标系2的z轴旋转

dh参数表
i | αi-1 | ai-1 | di | θi
1 | 0 | 0 | 0 | faiz
2 | 90° | -l | 0 | theta
3 | -90° | L | 0 | 0

可得到基座到末端的变换矩阵
T03 = T01 * T12 * T23
### 可得到已有的旋转矩阵

记号：左上标是参考系，左下标是本体系。${}^{b}_{a}R$ 表示 a 系相对 b 系的旋转矩阵，作用是把 a 系下的向量变到 b 系。

0 系 = 底盘坐标系 B，3 系 = 末端姿态坐标系 G。$\varphi$ 即 faiz，$\theta$ 即 theta。$c_\varphi=\cos\varphi$，$s_\varphi=\sin\varphi$，$c_\theta=\cos\theta$，$s_\theta=\sin\theta$。

**1. IMU 直接测得**

$$
{}^{W}_{G}R:\ \text{云台 IMU，云台到世界}\qquad
{}^{W}_{B}R:\ \text{底盘 IMU，底盘到世界}
$$

**2. 由 DH 参数表得到（每行 $R_x(\alpha_{i-1})\,R_z(\theta_i)$）**

$$
{}^{0}_{1}R = R_z(\varphi) =
\begin{bmatrix}
c_\varphi & -s_\varphi & 0\\
s_\varphi & c_\varphi & 0\\
0 & 0 & 1
\end{bmatrix}
$$

$$
{}^{1}_{2}R = R_x(90^\circ)\,R_z(\theta) =
\begin{bmatrix}
c_\theta & -s_\theta & 0\\
0 & 0 & -1\\
s_\theta & c_\theta & 0
\end{bmatrix}
$$

$$
{}^{2}_{3}R = R_x(-90^\circ)\,R_z(0) =
\begin{bmatrix}
1 & 0 & 0\\
0 & 0 & 1\\
0 & -1 & 0
\end{bmatrix}
$$

**3. 连乘得到**

$$
{}^{1}_{3}R = {}^{1}_{2}R\,{}^{2}_{3}R = R_y(-\theta) =
\begin{bmatrix}
c_\theta & 0 & -s_\theta\\
0 & 1 & 0\\
s_\theta & 0 & c_\theta
\end{bmatrix}
$$

$$
{}^{0}_{3}R = {}^{0}_{1}R\,{}^{1}_{2}R\,{}^{2}_{3}R = R_z(\varphi)\,R_y(-\theta) =
\begin{bmatrix}
c_\varphi c_\theta & -s_\varphi & -c_\varphi s_\theta\\
s_\varphi c_\theta & c_\varphi & -s_\varphi s_\theta\\
s_\theta & 0 & c_\theta
\end{bmatrix}
$$

${}^{0}_{3}R$ 的第一列 $[c_\varphi c_\theta,\ s_\varphi c_\theta,\ s_\theta]^T$ 是视轴在底盘系下的方向，$\theta>0$ 时 z 分量为正，即抬头。

**4. 两个 IMU 与关节角之间的关系**

$$
{}^{B}_{G}R = \left({}^{W}_{B}R\right)^{T}\,{}^{W}_{G}R
\qquad \text{两个 IMU 算出的云台相对底盘姿态}
$$

$$
{}^{B}_{G}R = {}^{0}_{3}R
\qquad \text{由关节角（编码器）算出的同一个量}
$$

两者应相等，可用来标定编码器零位或检测异常。

**5. 反解**

$$
{}^{W}_{B}R = {}^{W}_{G}R\,\left({}^{0}_{3}R\right)^{T}
\qquad \text{只有云台 IMU 时，由编码器反解底盘姿态}
$$

$$
{}^{W}_{G}R = {}^{W}_{B}R\,{}^{0}_{3}R
\qquad \text{只有底盘 IMU 时，正解云台姿态}
$$

$$
{}^{B}_{W}R = \left({}^{W}_{B}R\right)^{T}
\qquad \text{世界到底盘，用于把目标指向投到底盘系}
$$

### 末端目标->基座 & 末端云台->基座

视觉的末端目标形式为世界系下的表示，先左乘 $\left({}^{W}_{T}R\right)$ 再左乘 $\left({}^{W}_{B}R\right)^T$ 得到底盘系下的表示

云台末端左乘 $\left({}^{B}_{G}R\right)$，即为 $\left({}^{0}_{3}R\right)$，得到底盘系下的表示

### 控制目标

目标在底盘系下的表示为 ${}^{B}_{T}R = \left({}^{W}_{B}R\right)^T\,{}^{W}_{T}R$，云台末端在底盘系下的表示为 ${}^{B}_{G}R = {}^{0}_{3}R$。控制目标是让两者相同。

但二轴云台只有 $\varphi$、$\theta$ 两个自由度，而姿态矩阵有三个。目标的 Roll 绕 $x$ 轴，云台控制不了，所以不能要求整个矩阵相等，只要求 $x$ 轴方向一致，即取两个矩阵的第一列。

**1. 取 x 方向向量**

目标的指向向量（底盘系）：

$$
\boldsymbol{p}_B = {}^{B}_{T}R \begin{bmatrix}1\\0\\0\end{bmatrix}
= \left({}^{W}_{B}R\right)^T\,{}^{W}_{T}R \begin{bmatrix}1\\0\\0\end{bmatrix}
= \begin{bmatrix}p_x\\p_y\\p_z\end{bmatrix}
$$

如果视觉直接给世界系下的指向向量 $\boldsymbol{p}_W$，则 $\boldsymbol{p}_B = \left({}^{W}_{B}R\right)^T\boldsymbol{p}_W$，不用再乘 ${}^{W}_{T}R$。

云台的视轴向量（底盘系）：

$$
\boldsymbol{v}_B = {}^{0}_{3}R \begin{bmatrix}1\\0\\0\end{bmatrix}
= \begin{bmatrix}c_\varphi c_\theta\\ s_\varphi c_\theta\\ s_\theta\end{bmatrix}
$$

**2. 令两个向量相等，反解关节角**

$\boldsymbol{v}_B = \boldsymbol{p}_B$（旋转矩阵作用在单位向量上，$\boldsymbol{p}_B$ 本身就是单位向量，不用归一化；下面两个 atan2 也只和分量的比值有关，与长度无关）：

$$
\varphi_{cmd} = \operatorname{atan2}(p_y,\ p_x),\qquad
\theta_{cmd} = \operatorname{atan2}\!\left(p_z,\ \sqrt{p_x^2+p_y^2}\right)
$$

$\theta$ 用水平投影长度做分母，取值天然落在 $[-90^\circ, 90^\circ]$，不会出现云台倒立的解。

**3. 限位与奇异**

- pitch 有机械限位，在关节空间直接截断：$\theta_{cmd} \leftarrow \operatorname{clip}(\theta_{cmd},\ \theta_{min},\ \theta_{max})$。yaw 不截断，仍然对准目标所在的竖直平面，这是球面距离意义下的最优解。
- 目标在正上方或正下方时，$p_x,\,p_y\approx 0$，$\varphi$ 不确定。此时保持上一周期的 $\varphi_{cmd}$。

**4. 关节空间的串级控制**

姿态耦合和底盘扰动都已经在上面的反解里处理掉，剩下的是普通的关节空间跟随：

- 角度环：$e_\varphi = \operatorname{wrap}(\varphi_{cmd} - \varphi_{enc})$，wrap 映射到 $(-180^\circ, 180^\circ]$ 取最短路径。$e_\theta = \theta_{cmd} - \theta_{enc}$，不用 wrap。
- 速度环：反馈用电机编码器的相对角速度，输出电机力矩或电流。

底盘转动时，速度环抵抗不了扰动，是靠 $\varphi_{cmd}$、$\theta_{cmd}$ 随底盘姿态实时变化来补偿，所以反解要和姿态更新同频。有底盘 IMU 时，可以把底盘角速度投影到关节轴上作为速度环前馈。

## yaw 轴拉格朗日运动方程

暂时在底盘系下考虑：底盘静止、水平，{J1} 的 $z$ 轴竖直向上，把 {J1} 当作惯性系。

**1. 已知量**

- 广义坐标：$\varphi$（yaw），$\theta$（pitch）。
- 连杆 1（转台、叉臂、pitch 电机定子）：质心在 yaw 轴上，绕 yaw 轴的转动惯量 $J_1$（常数）。
- 连杆 2（云台本体加枪管）：质量 $m$，质心在 {J2} 系下为 $\boldsymbol c=[c_x,\ c_y,\ c_z]^T$，绕质心的惯量张量 ${}^{2}I_c$（在 {J2} 系下表示）。
- DH 给出的 {J2} 相对 {J1}：

$$
{}^{1}_{2}R=\begin{bmatrix}c_\theta&-s_\theta&0\\0&0&-1\\s_\theta&c_\theta&0\end{bmatrix},\qquad
\boldsymbol p={}^{1}\boldsymbol p_2=\begin{bmatrix}a_1\\0\\d_2\end{bmatrix}=\begin{bmatrix}-l\\0\\0\end{bmatrix}
$$

记 $\boldsymbol e_z=[0,0,1]^T$。

**2. 连杆 2 的角速度**

yaw 角速度 $\dot\varphi\,\boldsymbol e_z$ 先从 {J1} 系换到 {J2} 系，再加上 pitch 自己的转速：

$$
{}^{2}\boldsymbol\omega_2=\left({}^{1}_{2}R\right)^T\dot\varphi\,\boldsymbol e_z+\dot\theta\,\boldsymbol e_z
=\dot\varphi\,\boldsymbol u+\dot\theta\,\boldsymbol e_z,\qquad
\boldsymbol u=\left({}^{1}_{2}R\right)^T\boldsymbol e_z=\begin{bmatrix}s_\theta\\c_\theta\\0\end{bmatrix}
$$

$\boldsymbol u$ 是 yaw 轴方向在 {J2} 系下的表示，即 ${}^{1}_{2}R$ 第三行的转置。

**3. 连杆 2 质心的位置和速度（{J1} 系）**

$$
\boldsymbol r=\boldsymbol p+{}^{1}_{2}R\,\boldsymbol c=\begin{bmatrix}r_x\\r_y\\r_z\end{bmatrix}
=\begin{bmatrix}-l+c_x c_\theta-c_y s_\theta\\ -c_z\\ c_x s_\theta+c_y c_\theta\end{bmatrix}
$$

质心绕 yaw 轴转动，同时 pitch 改变它的位置：

$$
\boldsymbol v_c=\dot\varphi\,(\boldsymbol e_z\times\boldsymbol r)+\dot\theta\,\frac{\partial\boldsymbol r}{\partial\theta},\qquad
\frac{\partial\boldsymbol r}{\partial\theta}={}^{1}_{2}R\,(\boldsymbol e_z\times\boldsymbol c)
=\begin{bmatrix}-r_z\\ 0\\ r_x+l\end{bmatrix}
$$

这里 $\partial\boldsymbol r/\partial\theta$ 用到了 $\partial\,{}^{1}_{2}R/\partial\theta={}^{1}_{2}R\,[\boldsymbol e_z]_\times$（绕 {J2} 自身的 $z$ 轴转）。表达式里取了 $c_z=0$ 化简。

**4. 动能**

连杆 1 只绕 yaw 轴转：$T_1=\tfrac12J_1\dot\varphi^2$。

连杆 2 的动能是平动加转动：

$$
T_2=\tfrac12 m\,\boldsymbol v_c^T\boldsymbol v_c+\tfrac12\,{}^{2}\boldsymbol\omega_2^T\,{}^{2}I_c\,{}^{2}\boldsymbol\omega_2
$$

代入后按 $\dot\varphi^2$、$\dot\varphi\dot\theta$、$\dot\theta^2$ 整理：

$$
T=\tfrac12M_{\varphi\varphi}\dot\varphi^2+M_{\varphi\theta}\dot\varphi\dot\theta+\tfrac12M_{\theta\theta}\dot\theta^2
$$

各系数用矩阵写出（${}^{2}I_c$ 的元素记作 $I_{ij}$）：

$$
M_{\varphi\varphi}=J_{yaw}(\theta)=J_1+\boldsymbol u^T\,{}^{2}I_c\,\boldsymbol u+m\,(r_x^2+r_y^2)
$$

$$
M_{\varphi\theta}=\boldsymbol u^T\,{}^{2}I_c\,\boldsymbol e_z+m\,(\boldsymbol e_z\times\boldsymbol r)^T\frac{\partial\boldsymbol r}{\partial\theta}
=s_\theta I_{xz}+c_\theta I_{yz}-m\,c_z\,r_z
$$

$$
M_{\theta\theta}=\boldsymbol e_z^T\,{}^{2}I_c\,\boldsymbol e_z+m\left\|\frac{\partial\boldsymbol r}{\partial\theta}\right\|^2
=I_{zz}+m\,(c_x^2+c_y^2)
$$

其中平动部分 $(\boldsymbol e_z\times\boldsymbol r)^T\,\partial\boldsymbol r/\partial\theta=r_y r_z=-c_z r_z$。云台关于垂直 pitch 轴的中面对称时，$c_z=0$，$I_{xz}=I_{yz}=0$，所以 $M_{\varphi\theta}=0$，质量矩阵是对角的。下面按此处理。

$J_{yaw}$ 展开（${}^{2}I_c=\mathrm{diag}(I_x,I_y,I_z)$ 时）：

$$
J_{yaw}(\theta)=J_1+I_x s_\theta^2+I_y c_\theta^2+m\,(-l+c_x c_\theta-c_y s_\theta)^2
$$

**5. 势能**

高度只取 $z$ 分量，与 $\varphi$ 无关：

$$
V=m\,g\,r_z=m\,g\,(c_x s_\theta+c_y c_\theta)
$$

**6. 拉格朗日方程**

$L=T-V$，对 $\varphi$：

$$
\frac{d}{dt}\frac{\partial L}{\partial\dot\varphi}-\frac{\partial L}{\partial\varphi}=\tau_\varphi
$$

因为 $T$ 和 $V$ 都不含 $\varphi$，$\partial L/\partial\varphi=0$。$\partial L/\partial\dot\varphi=J_{yaw}(\theta)\,\dot\varphi$，对时间求导时 $J_{yaw}$ 通过 $\theta$ 随时间变化：

$$
\boxed{\,J_{yaw}(\theta)\,\ddot\varphi+\frac{dJ_{yaw}}{d\theta}\,\dot\theta\,\dot\varphi=\tau_\varphi\,}
$$

**7. $dJ_{yaw}/d\theta$ 的矩阵写法**

$\partial\boldsymbol u/\partial\theta=[c_\theta,\ -s_\theta,\ 0]^T$，$\partial\boldsymbol r/\partial\theta$ 见第 3 节：

$$
\frac{dJ_{yaw}}{d\theta}=2\,\boldsymbol u^T\,{}^{2}I_c\,\frac{\partial\boldsymbol u}{\partial\theta}+2m\left(r_x\frac{\partial r_x}{\partial\theta}+r_y\frac{\partial r_y}{\partial\theta}\right)
=(I_x-I_y)\sin2\theta-2m\,r_x\,r_z
$$

**8. 结果的含义**

- $J_{yaw}\ddot\varphi$：yaw 的惯性力矩，惯量随 pitch 角变化。
- $J'_{yaw}\,\dot\theta\,\dot\varphi$：pitch 转动时 yaw 惯量在变，角动量 $J_{yaw}\dot\varphi$ 要变化，这一项是 pitch 运动在 yaw 轴上产生的耦合力矩。
- 重力不出现在 yaw 方程里，因为底盘水平时重力方向与 yaw 轴平行。
- 检验：$\dot\theta=0$ 时退化为 $J\ddot\varphi=\tau_\varphi$。$\theta=0$ 且质心在轴上（$r_x=0$）时，$J'_{yaw}=0$，两轴解耦。
- 实际电机侧：$\tau_\varphi=n\,\tau_m-b\,\dot\varphi-\tau_c\,\mathrm{sgn}(\dot\varphi)$，减速比 $n$ 时把转子惯量折算后并入 $J_1$。

## 控制

上一节我们得到了关节的目标角，采用编码器闭环控制，接下来进行控制器的设计

对云台运动模型系统分析

首先看yaw轴

转动惯量来自连杆1和连杆2，连杆2与pitch轴转角相关

由图的质心分布测得连杆1的质心的位置以及绕关节坐标系1的转动惯量为J1，连杆2的质心位置以及绕关节坐标系2的转动惯量为J2，将J2转换到关节坐标系1下的转动惯量，得到yaw的等效转动惯量

