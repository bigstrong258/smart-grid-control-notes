# 《Optimization of Communication Network for Distributed Control of Wind Farm Equipped With Energy Storages》阅读记录

## 0. 阅读说明

本文是一篇发表在 *IEEE Transactions on Sustainable Energy* 上的论文，主题是配置分布式储能的风电场分布式控制及通信网络优化。

阅读本文时，可以先从电机控制和电力电子控制背景出发理解 DFIG 与 ES 的建模方式，再逐步补充后续涉及的现代控制理论、图论、通信拓扑优化和 ADMM 求解框架等内容。

---

## 1. DFIG 与 ES 系统结构

本文研究配置分布式储能的风电场。每台风机采用 DFIG，并在 DFIG 的直流母线上通过 DC/DC 变换器接入一个 ES 单元。

基本结构为：

| 部分 | 作用 |
|---|---|
| RSC | rotor-side converter，转子侧变流器，主要控制 DFIG 有功/无功 |
| GSC | grid-side converter，网侧变流器，主要维持直流母线与并网功率交换 |
| ES | energy storage，经 DC/DC 接入直流母线，参与有功协调 |

论文为降低 GSC 电流和损耗，假设正常运行下 GSC 不提供无功：

$$
Q_{\mathrm{g}}=0.
$$

因此风机无功主要由 DFIG 本体承担：

$$
Q_{\mathrm{w}}=Q_{\mathrm{d}}=Q_{\mathrm{s}}.
$$

---

## 2. DFIG 功率控制等效模型

Fig. 2 不是完整电磁暂态模型，而是服务于风电场级控制的 DFIG 功率动态等效模型。它可概括为：

$$
\text{功率外环 PI}
\rightarrow
\text{转子电流内环一阶等效}
\rightarrow
\text{电流到功率的静态映射}
\rightarrow
\text{功率反馈滤波}.
$$

后续状态空间推导中主要用到这些量：

| 变量/环节 | 含义 |
|---|---|
| $Q_{\mathrm{d}}^{\mathrm{ref}}$ | DFIG 无功参考，由风电场级无功分配给出 |
| $P_{\mathrm{d}}^{\mathrm{ref}}$ | DFIG 有功参考，通常由 MPPT 决定 |
| $i_{\mathrm{dr}}$ | 转子 d 轴电流，主要影响无功 |
| $i_{\mathrm{qr}}$ | 转子 q 轴电流，主要影响有功 |
| $1/(sT_{\mathrm{ir}}+1)$ | RSC 电流内环的一阶闭环等效 |
| $1/(sT_{\mathrm{fr}}+1)$ | DFIG 功率测量反馈滤波或延迟 |

注意：$s_{\mathrm{g}}$ 是 slip ratio，即转差率；PI 控制器中的 $s$ 是拉普拉斯算子，二者不是同一个量。

---

## 3. DFIG 有功功率与转子 q 轴电流

DFIG 定子直接并网，定子电角频率由电网决定。若 $s_{\mathrm{g}}$ 为转差率，则：

$$
p_{\mathrm{n}}\omega_{\mathrm{m}}=(1-s_{\mathrm{g}})\omega_{\mathrm{s}}.
$$

定子磁链定向下，电磁转矩近似为：

$$
T_{\mathrm{e}}=\frac{3}{2}p_{\mathrm{n}}\frac{L_{\mathrm{m}}}{L_{\mathrm{s}}}\psi_{\mathrm{s}}i_{\mathrm{qr}}.
$$

由：

$$
\omega_{\mathrm{m}}=\frac{(1-s_{\mathrm{g}})\omega_{\mathrm{s}}}{p_{\mathrm{n}}}
$$

代入 $P_{\mathrm{d}}=T_{\mathrm{e}}\omega_{\mathrm{m}}$，极对数 $p_{\mathrm{n}}$ 会抵消，因此得到论文使用的有功映射：

$$
P_{\mathrm{d}}=(1-s_{\mathrm{g}})\frac{3L_{\mathrm{m}}\omega_{\mathrm{s}}\psi_{\mathrm{s}}}{2L_{\mathrm{s}}}i_{\mathrm{qr}}.
$$

因此，$i_{\mathrm{qr}}$ 是 DFIG 有功功率动态模型中的主要控制通道。

---

## 4. DFIG 无功功率与转子 d 轴电流

Fig. 2 中 DFIG 无功功率写成：

$$
Q_{\mathrm{d}}=K_{\mathrm{Q}}i_{\mathrm{dr}}-Q_{\mathrm{m}}.
$$

其中：

$$
K_{\mathrm{Q}}=\frac{3L_{\mathrm{m}}\omega_{\mathrm{s}}\psi_{\mathrm{s}}}{2L_{\mathrm{s}}},
\quad
Q_{\mathrm{m}}=\frac{3\omega_{\mathrm{s}}\psi_{\mathrm{s}}^2}{2L_{\mathrm{s}}}.
$$

$Q_{\mathrm{m}}$ 表示建立定子磁链所需的励磁无功偏置。简要推导如下。定子磁链定向下：

$$
\psi_{\mathrm{sd}}=\psi_{\mathrm{s}},
\quad
\psi_{\mathrm{sq}}=0.
$$

定子 d 轴磁链满足：

$$
\psi_{\mathrm{sd}}=L_{\mathrm{s}}i_{\mathrm{sd}}+L_{\mathrm{m}}i_{\mathrm{dr}},
\quad
i_{\mathrm{sd}}=\frac{\psi_{\mathrm{s}}-L_{\mathrm{m}}i_{\mathrm{dr}}}{L_{\mathrm{s}}}.
$$

忽略定子电阻和磁链暂态时，近似有：

$$
v_{\mathrm{sd}}\approx 0,
\quad
v_{\mathrm{sq}}\approx \omega_{\mathrm{s}}\psi_{\mathrm{s}}.
$$

dq 坐标下三相瞬时无功可写成：

$$
Q=\frac{3}{2}\left(v_{\mathrm{sq}}i_{\mathrm{sd}}-v_{\mathrm{sd}}i_{\mathrm{sq}}\right).
$$

将上述近似和 $i_{\mathrm{sd}}$ 代入，并按“DFIG 对外输出无功为正”的符号约定整理，即得到 $Q_{\mathrm{d}}=K_{\mathrm{Q}}i_{\mathrm{dr}}-Q_{\mathrm{m}}$。因此，当 $i_{\mathrm{dr}}=0$ 时：

$$
Q_{\mathrm{d}}=-Q_{\mathrm{m}}.
$$

因此，$i_{\mathrm{dr}}$ 的作用是调节 DFIG 无功，并补偿异步机建立气隙磁场所需的励磁无功。

---

## 5. 功率反馈滤波环节

Fig. 2 中的反馈一阶环节表示功率测量滤波或测量延迟：

$$
Q_{\mathrm{d},\mathrm{f}}=\frac{1}{sT_{\mathrm{fr}}+1}Q_{\mathrm{d}},
\quad
P_{\mathrm{d},\mathrm{f}}=\frac{1}{sT_{\mathrm{fr}}+1}P_{\mathrm{d}}.
$$

因此功率外环比较的是滤波后的反馈量，例如：

$$
Q_{\mathrm{d}}^{\mathrm{ref}}-Q_{\mathrm{d},\mathrm{f}}.
$$

该一阶环节是后续状态空间模型中的动态状态来源之一。

---

## 6. DFIG 控制结构的基本类比

Fig. 2 的控制对象不是机械速度，而是并网 DFIG 的有功/无功功率。与常见电机控制结构的对应关系可以简化理解为：

| IPMSM 控制 | DFIG RSC 控制 |
|---|---|
| 速度外环 PI | 有功/无功功率外环 PI |
| 电流内环控制定子电流 | 电流内环控制转子电流 |
| $i_{\mathrm{q}}$ 主要控制转矩 | $i_{\mathrm{qr}}$ 主要控制有功 |
| $i_{\mathrm{d}}$ 调节磁链/弱磁 | $i_{\mathrm{dr}}$ 主要控制无功/励磁 |

这个类比只用于理解控制层级；后续推导仍以论文给出的 DFIG 功率等效模型为准。

---

## 7. ES 功率控制等效模型

Fig. 3 中 ES 通过 DC/DC 变换器控制充放电功率。其等效结构为：

$$
\text{功率外环 PI}
\rightarrow
\text{DC/DC 电感电流内环一阶等效}
\rightarrow
\text{功率反馈滤波}.
$$

主要变量为：

| 变量/环节 | 含义 |
|---|---|
| $P_{\mathrm{e}}^{\mathrm{ch},\mathrm{ref}}<0$ | ES 充电功率参考 |
| $P_{\mathrm{e}}^{\mathrm{dis},\mathrm{ref}}>0$ | ES 放电功率参考 |
| $i_{\mathrm{L}}^{\mathrm{ref}}$ | DC/DC 电感电流参考 |
| $1/(sT_{\mathrm{id}}+1)$ | DC/DC 电流内环的一阶闭环等效 |
| $1/(sT_{\mathrm{fd}}+1)$ | ES 功率反馈滤波或测量延迟 |

DC 侧功率近似为：

$$
P_{\mathrm{e}}=U_{\mathrm{e}}i_{\mathrm{L}}.
$$

因此 ES 功率环看到的等效对象可写成：

$$
G_{\mathrm{ES}}(s)\approx\frac{U_{\mathrm{e}}}{sT_{\mathrm{id}}+1}.
$$

ES 功率环直接控制的是 $P_{\mathrm{e}}$，而 SOC 是上层一致性控制中的慢状态变量。

---

## 8. DFIG 无功功率约束

DFIG 的无功参考需要满足视在功率容量约束：

$$
|Q_{\mathrm{d},i}^{\mathrm{ref}}|
\leq
\sqrt{S_{\mathrm{d},\mathrm{N}}^2-P_{\mathrm{d},i}^2}
$$

其来源是：

$$
P_{\mathrm{d},i}^2+Q_{\mathrm{d},i}^2\leq S_{\mathrm{d},\mathrm{N}}^2
$$

因此，当前有功 $P_{\mathrm{d},i}$ 越大，DFIG 剩余可用无功容量越小。该约束会影响后续风电场级无功分配中每台 DFIG 可承担的 $Q_{\mathrm{d},i}^{\mathrm{ref}}$ 上限。

---

## 9. ES 充放电功率约束

ES 的功率约束同时受到 DC/DC 变换器容量和 GSC 剩余容量限制。

符号约定为：

$$
P_{\mathrm{e}}^{\mathrm{dis},\mathrm{ref}}>0,
\quad
P_{\mathrm{e}}^{\mathrm{ch},\mathrm{ref}}<0.
$$

对应约束为：

$$
\begin{cases}
\displaystyle
|P_{\mathrm{e},i}^{\mathrm{dis},\mathrm{ref}}|
\leq
\min\left(P_{\mathrm{DC/DC}}^{\mathrm{lim}},P_{\mathrm{GSC}}^{\mathrm{lim}}-P_{\mathrm{r},i}\right),
\\[8pt]
\displaystyle
|P_{\mathrm{e},i}^{\mathrm{ch},\mathrm{ref}}|
\leq
\min\left(P_{\mathrm{DC/DC}}^{\mathrm{lim}},P_{\mathrm{GSC}}^{\mathrm{lim}}+P_{\mathrm{r},i}\right).
\end{cases}
$$

也就是说，ES 的充放电功率既不能超过 DC/DC 变换器容量，也不能使 GSC 的功率交换超过容量限制。放电时 ES 向直流母线送出功率，会占用 GSC 剩余容量；充电时 ES 从直流母线吸收功率，与转子侧功率在 GSC 负担上的方向相反，因此式中分别出现 $P_{\mathrm{GSC}}^{\mathrm{lim}}-P_{\mathrm{r},i}$ 和 $P_{\mathrm{GSC}}^{\mathrm{lim}}+P_{\mathrm{r},i}$。

---

## 10. 风电场级有功分配逻辑

本文中 DFIG 有功参考 $P_{\mathrm{d}}^{\mathrm{ref}}$ 主要由 MPPT 决定。若 TSO 给出风电场总有功指令 $P_{\mathrm{wf}}^{\mathrm{ref}}$，则 ES 侧承担剩余有功：

$$
P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}
=
P_{\mathrm{wf}}^{\mathrm{ref}}
-\sum_{i=1}^{N}P_{\mathrm{d},i}
$$

对应风电场有功平衡为：

$$
\Delta P
=
P_{\mathrm{wf}}^{\mathrm{ref}}
-\sum_{i=1}^{N}
(P_{\mathrm{d},i}+P_{\mathrm{e},i})
=0
$$

因此，本节的分工可以概括为：DFIG 尽量保持 MPPT，ES 负责补偿风电场有功偏差。这也是本文将 ES 接入 DFIG 直流母线的目的之一：在不显著牺牲风机 MPPT 的情况下，提高风电场有功调节和平滑能力。

---

## 11. ES 一致性变量的作用

第 10 节只确定了 ES 总有功任务：

$$
\sum_{i=1}^{N}P_{\mathrm{e},i}=P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}.
$$

但该式没有说明每个 ES 应承担多少功率。若只做平均功率分配，会忽略 SOC 差异；若只追求 SOC 一致，又可能不能快速跟踪风电场有功指令。因此，论文引入兼顾功率和 SOC 的一致性变量 [27，Khazaei et al., 2020, “Consensus-Based Demand Response of PMSG Wind Turbines With Distributed Energy Storage Considering Capability Curves”]：

$$
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}.
$$

其中，$P_{\mathrm{e},i}$ 是第 $i$ 个 ES 的有功功率，$S_{\mathrm{e},i}$ 是其 SOC，$K_1$ 和 $K_2$ 是权重系数。

---

### 11.1 一致性目标

论文要求各 ES 的一致性变量收敛到同一值：

$$
E_{\mathrm{e},1}=E_{\mathrm{e},2}=\cdots=E_{\mathrm{e},N}.
$$

这并不等价于所有 ES 功率完全相同，也不等价于 SOC 瞬间完全相同；它表示各 ES 在“当前功率分担”和“剩余能量状态”组合后的指标上达成一致。

> SOC（State of Charge，荷电状态）表示储能剩余能量占额定容量的比例。若约定 $P_{\mathrm{e}}>0$ 表示放电，则 SOC 会下降；$P_{\mathrm{e}}<0$ 表示充电，则 SOC 会上升。引入 SOC 是为了避免只按功率平均分配而造成部分 ES 过度充放电。

---

### 11.2 从公式看功率分配

若所有 ES 最终满足：

$$
K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}=E_{\mathrm{e}}^\ast,
$$

则：

$$
P_{\mathrm{e},i}=\frac{E_{\mathrm{e}}^\ast}{K_1}-\frac{K_2}{K_1}S_{\mathrm{e},i}.
$$

再结合总功率约束：

$$
\sum_{i=1}^{N}P_{\mathrm{e},i}=P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}},
$$

可得到：

$$
P_{\mathrm{e},i}=\frac{P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}}{N}-\frac{K_2}{K_1}\left(S_{\mathrm{e},i}-\bar S_{\mathrm{e}}\right),
\quad
\bar S_{\mathrm{e}}=\frac{1}{N}\sum_{i=1}^{N}S_{\mathrm{e},i}.
$$

因此，ES 的有功分配可以看成“平均功率分担项 + SOC 偏差修正项”。$K_2/K_1$ 决定 SOC 偏差对功率分配的影响强度和方向；其符号应与 $P_{\mathrm{e}}$ 的充放电符号约定一致。

---

### 11.3 与分布式一致性控制的关系

引入 $E_{\mathrm{e},i}$ 后，每个 ES 只需要和通信邻居交换一致性变量或相关状态信息，并根据邻居差异：

$$
E_{\mathrm{e},j}-E_{\mathrm{e},i}
$$

调整自身有功参考，使 $E_{\mathrm{e},i}$ 逐步趋同。在 leader-follower 结构下，leader 节点还接收全场有功偏差 $\Delta P$，从而保证 ES 既能内部协调 SOC，又能共同完成风电场总有功指令。

因此，$E_{\mathrm{e},i}$ 的作用可以概括为：把快速功率支撑和慢速 SOC 管理统一到同一个分布式一致性控制变量中。

---

## 12. 风电场级无功分配逻辑

本文中 ES 只提供有功支撑，不参与无功调节。因此风电场无功偏差由各台 DFIG 共同承担。风电场级无功平衡为：

$$
Q_{\mathrm{wf},\mathrm{d}}^{\mathrm{ref}}=Q_{\mathrm{wf}}^{\mathrm{ref}}
$$

$$
\Delta Q
=
Q_{\mathrm{wf}}^{\mathrm{ref}}
-
\sum_{i=1}^{N}Q_{\mathrm{d},i}
=0
$$

其中，$Q_{\mathrm{wf}}^{\mathrm{ref}}$ 是风电场无功调度指令，$Q_{\mathrm{d},i}$ 是第 $i$ 台 DFIG 的无功输出。每台 DFIG 的无功参考还受容量约束：

$$
|Q_{\mathrm{d},i}^{\mathrm{ref}}|
\leq
\sqrt{S_{\mathrm{d},\mathrm{N}}^2-P_{\mathrm{d},i}^2}
$$

因此，无功分配不宜简单平均，而应按各机组的可调无功能力进行比例分担。论文定义 DFIG 无功一致性变量：

$$
E_{\mathrm{d},i}=
\frac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}}
$$

其中，$A_{\mathrm{d},i}$ 表示第 $i$ 台 DFIG 的可调无功空间。要求：

$$
E_{\mathrm{d},1}=E_{\mathrm{d},2}=\cdots=E_{\mathrm{d},N}
$$

即：

$$
\frac{Q_{\mathrm{d},1}}{A_{\mathrm{d},1}}
=
\frac{Q_{\mathrm{d},2}}{A_{\mathrm{d},2}}
=\cdots
=\frac{Q_{\mathrm{d},N}}{A_{\mathrm{d},N}}
$$

该条件表示各 DFIG 的无功能力利用率一致，而不是无功输出值完全相同。若一致性变量收敛到 $\eta$，则：

$$
Q_{\mathrm{d},i}=\eta A_{\mathrm{d},i}
$$

即第 $i$ 台 DFIG 分担的无功与其可调空间成正比。论文中 $A_{\mathrm{d},i}$ 根据无功偏差方向取：

$$
A_{\mathrm{d},i}
=
\begin{cases}
Q_{\mathrm{d},i,0}, & \Delta Q<0\\
\sqrt{S_{\mathrm{d},\mathrm{N}}^2-P_{\mathrm{d},i}^2}, & \Delta Q>0
\end{cases}
$$

当 $\Delta Q>0$ 时，风电场无功不足，需要增加无功输出，$A_{\mathrm{d},i}$ 由剩余视在功率容量决定；当 $\Delta Q<0$ 时，风电场无功偏多，需要减少无功输出，$A_{\mathrm{d},i}$ 与原有无功输出 $Q_{\mathrm{d},i,0}$ 有关。这样可以避免简单平均分配导致部分 DFIG 过早触及无功容量边界。

---

## 13. 状态变量选取与 PI 积分状态的理解

阅读 Fig. 2 和 Fig. 3 的状态空间模型时，首先要明确：$Q_{\mathrm{d},\mathrm{int},i}$ 和 $P_{\mathrm{e},\mathrm{int},i}$ 中的 $\mathrm{int}$ 不是 initial，而是 integral，表示 PI 控制器的积分状态。

---

### 13.1 $\mathrm{int}$ 下标表示积分状态

本文中的两个积分状态为：

$$
\begin{cases}
\displaystyle
Q_{\mathrm{d},\mathrm{int},i}
=
\int
\left(
Q_{\mathrm{d},i}^{\mathrm{ref}}-Q_{\mathrm{d},i}
\right)\,\mathrm{d}t,\\[8pt]
\displaystyle
P_{\mathrm{e},\mathrm{int},i}
=
\int
\left(
P_{\mathrm{e},i}^{\mathrm{ref}}-P_{\mathrm{e},i}
\right)\,\mathrm{d}t.
\end{cases}
$$

它们不是初始值，而是控制器内部的误差累计量。它们当然也有初始条件，例如 $Q_{\mathrm{d},\mathrm{int},i}(0)$ 和 $P_{\mathrm{e},\mathrm{int},i}(0)$，但物理含义是“积分状态”。

PI 控制器需要把积分项作为状态变量，是因为：

$$
u_{\mathrm{c}}=k_{\mathrm{p}}e+k_{\mathrm{i}}z,
\quad
\dot z=e.
$$

仅知道当前误差 $e(t)$ 不能确定控制器输出，还必须知道历史误差累计量 $z$。因此，若要把闭环系统写成一阶状态空间形式，就必须把 PI 积分状态放进状态向量。

---

### 13.2 状态变量选取规则

本文状态变量的选取遵循一个基本原则：凡是具有动态记忆、参与反馈控制或进入一致性变量的量，都应保留为状态。具体包括：

1. 一阶动态环节的输出，例如功率反馈滤波、电流内环等效输出；
2. 控制器内部动态，例如 PI 积分状态；
3. 物理能量状态，例如 ES 的 SOC；
4. 后续一致性控制或约束中显式使用的变量，例如 $Q_{\mathrm{d},i}$、$P_{\mathrm{e},i}$、$S_{\mathrm{e},i}$。

这样选取状态后，系统才能整理成一阶微分方程组，并进一步写成矩阵形式。

---

### 13.3 本文选取的状态变量

本文把第 $i$ 台 WT 的状态变量选为：

$$
x_i=
\begin{bmatrix}
Q_{\mathrm{d},i}&
Q_{\mathrm{d},\mathrm{int},i}&
i_{\mathrm{dr},i}&
P_{\mathrm{e},i}&
P_{\mathrm{e},\mathrm{int},i}&
i_{\mathrm{L},i}&
S_{\mathrm{e},i}
\end{bmatrix}^{\mathrm{T}}.
$$

其中前三个状态来自 DFIG 无功通道：

$$
\begin{bmatrix}
Q_{\mathrm{d},i}&
Q_{\mathrm{d},\mathrm{int},i}&
i_{\mathrm{dr},i}
\end{bmatrix}^{\mathrm{T}},
$$

后四个状态来自 ES 有功通道：

$$
\begin{bmatrix}
P_{\mathrm{e},i}&
P_{\mathrm{e},\mathrm{int},i}&
i_{\mathrm{L},i}&
S_{\mathrm{e},i}
\end{bmatrix}^{\mathrm{T}}.
$$

这个选择是为后续把 Fig. 2、Fig. 3 的底层动态与上层一致性控制统一写成矩阵形式做准备。

---

### 13.4 各状态变量的来源

各状态变量与 Fig. 2、Fig. 3 的动态环节对应如下：

| 状态变量 | 来源 | 典型动态关系 |
|---|---|---|
| $Q_{\mathrm{d},i}$ | DFIG 无功反馈滤波，也是 $E_{\mathrm{d},i}$ 的组成量 | $T_{\mathrm{fr}}\dot Q_{\mathrm{d},i}+Q_{\mathrm{d},i}=K_{\mathrm{Q}}i_{\mathrm{dr},i}$ |
| $Q_{\mathrm{d},\mathrm{int},i}$ | DFIG 无功 PI 积分状态 | $\dot Q_{\mathrm{d},\mathrm{int},i}=Q_{\mathrm{d},i}^{\mathrm{ref}}-Q_{\mathrm{d},i}$ |
| $i_{\mathrm{dr},i}$ | RSC 转子 d 轴电流内环状态 | $T_{\mathrm{ir}}\dot i_{\mathrm{dr},i}+i_{\mathrm{dr},i}=i_{\mathrm{dr},i}^{\mathrm{ref}}$ |
| $P_{\mathrm{e},i}$ | ES 有功反馈滤波，也是 $E_{\mathrm{e},i}$ 的组成量 | $T_{\mathrm{fd}}\dot P_{\mathrm{e},i}+P_{\mathrm{e},i}=U_{\mathrm{e}}i_{\mathrm{L},i}$ |
| $P_{\mathrm{e},\mathrm{int},i}$ | ES 有功 PI 积分状态 | $\dot P_{\mathrm{e},\mathrm{int},i}=P_{\mathrm{e},i}^{\mathrm{ref}}-P_{\mathrm{e},i}$ |
| $i_{\mathrm{L},i}$ | DC/DC 电感电流内环状态 | $T_{\mathrm{id}}\dot i_{\mathrm{L},i}+i_{\mathrm{L},i}=i_{\mathrm{L},i}^{\mathrm{ref}}$ |
| $S_{\mathrm{e},i}$ | ES 的 SOC 能量状态，也是 $E_{\mathrm{e},i}$ 的组成量 | $\dot S_{\mathrm{e},i}=-P_{\mathrm{e},i}$ |

其中：

$$
K_{\mathrm{Q}}=\frac{3L_{\mathrm{m}}\omega_{\mathrm{s}}\psi_{\mathrm{s}}}{2L_{\mathrm{s}}},
\quad
E_{\mathrm{d},i}=\frac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}},
\quad
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}.
$$

这些关系式正是附录 A 中构造 $A_i$、$B_i$ 以及后续一致性反馈矩阵 $C_{ij}$ 的来源。

---

### 13.5 为什么没有把 DFIG 有功环状态放进去

如果只看 Fig. 2(b)，DFIG 有功环也可以写成类似状态：

$$
\begin{bmatrix}
P_{\mathrm{d},i}&
P_{\mathrm{d},\mathrm{int},i}&
i_{\mathrm{qr},i}
\end{bmatrix}^{\mathrm{T}}
$$

但本文用于通信拓扑优化的上层一致性控制主要生成 $Q_{\mathrm{d},i}^{\mathrm{ref}}$ 和 $P_{\mathrm{e},i}^{\mathrm{ref}}$。DFIG 有功参考由 MPPT 给出：

$$
P_{\mathrm{d},i}^{\mathrm{ref}}=P_{\mathrm{MPPT},i}
$$

因此，$P_{\mathrm{d},i}$ 更多由风速、MPPT 策略和外部环境决定，而不是通信网络直接分配的状态反馈对象。

$P_{\mathrm{d},i}$ 在本文中并没有消失，它仍然影响 DFIG 的无功能力约束：

$$
|Q_{\mathrm{d},i}^{\mathrm{ref}}|
\leq
\sqrt{S_{\mathrm{d},\mathrm{N}}^2-P_{\mathrm{d},i}^2}
$$

也影响风电场有功平衡：

$$
P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}
=
P_{\mathrm{wf}}^{\mathrm{ref}}
-\sum_{i=1}^{N}P_{\mathrm{d},i}
$$

但由于 $P_{\mathrm{d},i}^{\mathrm{ref}}$ 不由通信一致性控制生成，作者在用于通信拓扑优化的状态空间模型中主要保留 DFIG 无功通道和 ES 有功通道。

### 13.6 为什么可以写成式 (9) 的状态空间形式

前面已经说明，本文保留的状态变量都来自一阶动态环节、PI 积分状态或 SOC 能量状态。因此，在选定 $x_i$ 后，单台 WT 的底层动态可以整理为论文式 (9) 的第一行：

$$
\dot x_i=A_i x_i+B_i u_i.
$$

这里 $A_i$ 收集本机内部动态系数，例如功率滤波时间常数、电流内环时间常数、PI 参数、电流到功率的静态增益以及 SOC 方程；$B_i$ 描述上层参考输入如何进入底层功率环。论文将 $u_i$ 记为风电场级一致性控制生成的参考变化率：

$$
u_i=
\begin{bmatrix}
\dot Q_{\mathrm{d},i}^{\mathrm{ref}}\\
\dot P_{\mathrm{e},i}^{\mathrm{ref}}
\end{bmatrix}
$$

第二行来自上层一致性控制。DFIG 无功分配和 ES 有功分配分别使用一致性变量：

$$
\begin{cases}
\displaystyle
E_{\mathrm{d},i}=\dfrac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}},\\[8pt]
\displaystyle
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}.
\end{cases}
$$

在给定运行点或给定无功可调容量 $A_{\mathrm{d},i}$ 的情况下，这两个量都可以看成状态变量的线性组合。典型一致性控制项为：

$$
\sum_{j\in\vartheta_i}
(E_j-E_i)
$$

由于 $E_i$ 和 $E_j$ 可由对应节点状态表示，邻居差异项最终可以整理为状态反馈形式：

$$
u_i=-\sum_{j\in\vartheta_i}C_{ij}x_j
$$

其中，$C_{ij}$ 不是底层物理参数矩阵，而是由一致性控制律、通信邻接关系、无功可调空间 $A_{\mathrm{d},i}$、ES 权重 $K_1,K_2$ 以及 leader-follower 结构共同决定的反馈矩阵。负号表示反馈方向是减小节点间一致性误差。

因此，式 (9) 本质上是在一个表达式中连接两层模型：

$$
\begin{cases}
\displaystyle
\dot x_i=A_i x_i+B_i u_i,
&
\text{本机 DFIG/ES 底层动态},\\[6pt]
\displaystyle
u_i=-\sum_{j\in\vartheta_i}C_{ij}x_j,
&
\text{基于邻居信息的一致性反馈}.
\end{cases}
$$

在这个形式下，通信拓扑通过邻居集合 $\vartheta_i$ 和矩阵 $C_{ij}$ 进入控制系统。后续把所有 WT 的状态堆叠起来，就可以得到整个风电场的闭环矩阵，并分析通信拓扑对收敛速度、连通性和鲁棒性的影响。

从建模角度看，式 (9) 依赖以下近似：

1. 底层功率环被近似为线性一阶动态；
2. PI 控制器的积分项被显式加入状态变量；
3. 电流到功率的映射在当前工作点附近被视为线性关系；
4. 常值项和运行点偏置可以通过工作点平移或小信号建模吸收；
5. 一致性变量 $E_{\mathrm{d},i}$ 和 $E_{\mathrm{e},i}$ 可在给定工况下视为状态变量的线性组合；
6. 通信一致性控制律由邻居状态差构成，因此可以整理为状态反馈形式。

所以，式 (9) 可以看作是把“设备底层控制动态”和“风电场级分布式一致性控制”连接起来的中间模型。后续通信网络优化之所以能够转化为闭环矩阵和图论问题，正是因为该式把通信邻接关系转化成了状态反馈矩阵的一部分。

### 13.7 附录 A 中单台 WT 状态空间矩阵的来源

附录 A 给出的矩阵本质上是把 Fig. 2 和 Fig. 3 中的各个一阶动态环节逐行写成状态方程后，再按状态变量顺序整理得到的结果。

>严格说，论文式 (9) 将输入记为 $\dot Q_{\mathrm{d},i}^{\mathrm{ref}}$ 和 $\dot P_{\mathrm{e},i}^{\mathrm{ref}}$，但从 Fig. 2、Fig. 3 的 PI 框图及附录 A 中 $B_i$ 的结构看，$B_i$ 的各项更符合“参考值进入误差环节”的传统 PI 写法。因此本文笔记在解释 $A_i$、$B_i$ 的来源时，按 $Q_{\mathrm{d},i}^{\mathrm{ref}}$ 和 $P_{\mathrm{e},i}^{\mathrm{ref}}$ 进入底层 PI 环节来理解；而在解释一致性控制律和全场闭环矩阵时，再保留论文中“参考变化率由一致性控制生成”的记号。

---

#### 13.7.1 为什么 $A_i$ 是块对角矩阵

附录 A 中：

$$
A_i=
\begin{bmatrix}
A_{i,1}&0\\
0&A_{i,2}
\end{bmatrix}
$$

其中，$A_{i,1}$ 对应 DFIG 无功通道，状态顺序为：

$$
\begin{bmatrix}
Q_{\mathrm{d},i}&
Q_{\mathrm{d},\mathrm{int},i}&
i_{\mathrm{dr},i}
\end{bmatrix}^{\mathrm{T}}
$$

$A_{i,2}$ 对应 ES 有功通道，状态顺序为：

$$
\begin{bmatrix}
P_{\mathrm{e},i}&
P_{\mathrm{e},\mathrm{int},i}&
i_{\mathrm{L},i}&
S_{\mathrm{e},i}
\end{bmatrix}^{\mathrm{T}}
$$

块对角结构表示：单台 WT 的底层模型中，DFIG 无功通道和 ES 有功通道被视为两个相对独立的线性子系统；二者的协调关系不放在 $A_i$ 中，而是通过上层一致性输入 $u_i$ 体现。

---

#### 13.7.2 $A_{i,1}$ 的来源：DFIG 无功通道

令：

$$
K_{\mathrm{Q}}
=
\frac{3L_{\mathrm{m}}\psi_{\mathrm{s}}\omega_{\mathrm{s}}}{2L_{\mathrm{s}}}
$$

DFIG 无功通道的三个状态为 $Q_{\mathrm{d},i}$、$Q_{\mathrm{d},\mathrm{int},i}$、$i_{\mathrm{dr},i}$。对应动态为：

$$
\begin{cases}
\displaystyle
\dot Q_{\mathrm{d},i}
=
-\dfrac{1}{T_{\mathrm{fr}}}Q_{\mathrm{d},i}
+
\dfrac{K_{\mathrm{Q}}}{T_{\mathrm{fr}}}i_{\mathrm{dr},i},\\[8pt]
\displaystyle
\dot Q_{\mathrm{d},\mathrm{int},i}
=
Q_{\mathrm{d},i}^{\mathrm{ref}}
-
Q_{\mathrm{d},i},\\[8pt]
\displaystyle
\dot i_{\mathrm{dr},i}
=
-\dfrac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}Q_{\mathrm{d},i}
+
\dfrac{k_{\mathrm{r},\mathrm{i}}^{\mathrm{q}}}{T_{\mathrm{ir}}}Q_{\mathrm{d},\mathrm{int},i}
-
\dfrac{1}{T_{\mathrm{ir}}}i_{\mathrm{dr},i}
+
\dfrac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}Q_{\mathrm{d},i}^{\mathrm{ref}}.
\end{cases}
$$

第一行来自无功功率反馈滤波；第二行来自无功 PI 积分状态；第三行来自“无功 PI 输出 $i_{\mathrm{dr},i}^{\mathrm{ref}}$ + 转子 d 轴电流内环”：

$$
i_{\mathrm{dr},i}^{\mathrm{ref}}
=
k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}
\left(
Q_{\mathrm{d},i}^{\mathrm{ref}}-Q_{\mathrm{d},i}
\right)
+
k_{\mathrm{r},\mathrm{i}}^{\mathrm{q}}Q_{\mathrm{d},\mathrm{int},i},
\quad
T_{\mathrm{ir}}\dot i_{\mathrm{dr},i}+i_{\mathrm{dr},i}=i_{\mathrm{dr},i}^{\mathrm{ref}}.
$$

把状态项收集到 $A_{i,1}$，把参考输入 $Q_{\mathrm{d},i}^{\mathrm{ref}}$ 收集到 $B_i$，即可得到：

$$
A_{i,1}
=
\begin{bmatrix}
-\frac{1}{T_{\mathrm{fr}}} & 0 & \frac{K_{\mathrm{Q}}}{T_{\mathrm{fr}}}\\
-1 & 0 & 0\\
-\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}} & \frac{k_{\mathrm{r},\mathrm{i}}^{\mathrm{q}}}{T_{\mathrm{ir}}} & -\frac{1}{T_{\mathrm{ir}}}
\end{bmatrix}
$$

这三行分别对应无功功率滤波、无功 PI 积分状态和转子 d 轴电流内环。

---

#### 13.7.3 $A_{i,2}$ 的来源：ES 有功通道

ES 有功通道的四个状态为 $P_{\mathrm{e},i}$、$P_{\mathrm{e},\mathrm{int},i}$、$i_{\mathrm{L},i}$、$S_{\mathrm{e},i}$。对应动态为：

$$
\begin{cases}
\displaystyle
\dot P_{\mathrm{e},i}
=
-\dfrac{1}{T_{\mathrm{fd}}}P_{\mathrm{e},i}
+
\dfrac{U_{\mathrm{e}}}{T_{\mathrm{fd}}}i_{\mathrm{L},i},\\[8pt]
\displaystyle
\dot P_{\mathrm{e},\mathrm{int},i}
=
P_{\mathrm{e},i}^{\mathrm{ref}}
-
P_{\mathrm{e},i},\\[8pt]
\displaystyle
\dot i_{\mathrm{L},i}
=
-\dfrac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}P_{\mathrm{e},i}
+
\dfrac{k_{\mathrm{d},\mathrm{i}}^{\mathrm{p}}}{T_{\mathrm{id}}}P_{\mathrm{e},\mathrm{int},i}
-
\dfrac{1}{T_{\mathrm{id}}}i_{\mathrm{L},i}
+
\dfrac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}P_{\mathrm{e},i}^{\mathrm{ref}},\\[8pt]
\displaystyle
\dot S_{\mathrm{e},i}=-P_{\mathrm{e},i}.
\end{cases}
$$

第一行来自 ES 功率反馈滤波，$U_{\mathrm{e}}$ 是当前充/放电工况下的等效端电压；第二行来自 ES 有功 PI 积分状态；第三行来自“有功 PI 输出 $i_{\mathrm{L},i}^{\mathrm{ref}}$ + DC/DC 电流内环”：

$$
i_{\mathrm{L},i}^{\mathrm{ref}}
=
k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}
\left(
P_{\mathrm{e},i}^{\mathrm{ref}}-P_{\mathrm{e},i}
\right)
+
k_{\mathrm{d},\mathrm{i}}^{\mathrm{p}}P_{\mathrm{e},\mathrm{int},i},
\quad
T_{\mathrm{id}}\dot i_{\mathrm{L},i}+i_{\mathrm{L},i}=i_{\mathrm{L},i}^{\mathrm{ref}}.
$$

第四行是归一化 SOC 动态。若保留额定能量容量 $E_{\mathrm{N}}$，更一般地可写为 $\dot S_{\mathrm{e},i}=-P_{\mathrm{e},i}/E_{\mathrm{N}}$；本文将容量基值吸收到标幺化中，因此得到 $\dot S_{\mathrm{e},i}=-P_{\mathrm{e},i}$。当 $P_{\mathrm{e},i}>0$ 时 ES 放电，SOC 下降；当 $P_{\mathrm{e},i}<0$ 时 ES 充电，SOC 上升。

收集状态项得到：

$$
A_{i,2}
=
\begin{bmatrix}
-\frac{1}{T_{\mathrm{fd}}} & 0 & \frac{U_{\mathrm{e}}}{T_{\mathrm{fd}}} & 0\\
-1 & 0 & 0 & 0\\
-\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}} & \frac{k_{\mathrm{d},\mathrm{i}}^{\mathrm{p}}}{T_{\mathrm{id}}} & -\frac{1}{T_{\mathrm{id}}} & 0\\
-1 & 0 & 0 & 0
\end{bmatrix}
$$

这四行分别对应 ES 有功功率滤波、ES 有功 PI 积分状态、DC/DC 电感电流内环和 SOC 能量状态。

---

#### 13.7.4 $B_i$ 的来源：参考指令输入

参考输入项没有放进 $A_{i,1}$ 或 $A_{i,2}$，而是统一放进 $B_i$。按 7 维状态顺序，附录 A 给出：

$$
B_i=
\begin{bmatrix}
0&0\\
1&0\\
\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}&0\\
0&0\\
0&1\\
0&\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}\\
0&0
\end{bmatrix}
$$

第一列对应 DFIG 无功参考：它进入 $\dot Q_{\mathrm{d},\mathrm{int},i}$ 的系数为 $1$，并通过 PI 比例环节进入 $\dot i_{\mathrm{dr},i}$，系数为 $k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}/T_{\mathrm{ir}}$。第二列对应 ES 有功参考：它进入 $\dot P_{\mathrm{e},\mathrm{int},i}$ 的系数为 $1$，并通过 PI 比例环节进入 $\dot i_{\mathrm{L},i}$，系数为 $k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}/T_{\mathrm{id}}$。功率滤波状态和 SOC 行没有直接参考输入，所以对应元素为 $0$。

---

#### 13.7.5 $C_{ij}$ 的来源：一致性控制律

附录 A 中的 $C_{ij}$ 不是设备物理参数，而是由一致性控制律整理得到的通信反馈矩阵。第 $i$ 台机组的上层输入由两类信息构成：邻居间一致性变量差异，以及 leader 节点接收的全局功率偏差。典型形式为：

$$
\dot Q_{\mathrm{d},i}^{\mathrm{ref}}
=
-c_1
\sum_{j\in\vartheta_i}
\left(
E_{\mathrm{d},i}-E_{\mathrm{d},j}
\right)
+
c_0^{\mathrm{Q}}M_{i0}\Delta Q
$$

$$
\dot P_{\mathrm{e},i}^{\mathrm{ref}}
=
-c_2
\sum_{j\in\vartheta_i}
\left(
E_{\mathrm{e},i}-E_{\mathrm{e},j}
\right)
+
c_0^{\mathrm{P}}M_{i0}\Delta P
$$

其中，$\vartheta_i$ 是第 $i$ 台 WT 的通信邻居集合，$M_{i0}=1$ 表示该节点为 leader，否则为 $0$。一致性变量为：

$$
\begin{cases}
\displaystyle
E_{\mathrm{d},i}=\dfrac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}},\\[8pt]
\displaystyle
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}.
\end{cases}
$$

由于 $Q_{\mathrm{d},i}$、$P_{\mathrm{e},i}$ 和 $S_{\mathrm{e},i}$ 分别是状态向量 $x_i$ 的第 1、第 4 和第 7 个元素，因此一致性变量差可展开为：

$$
\begin{cases}
\displaystyle
E_{\mathrm{d},i}-E_{\mathrm{d},j}
=
\frac{1}{A_{\mathrm{d},i}}Q_{\mathrm{d},i}
-\frac{1}{A_{\mathrm{d},j}}Q_{\mathrm{d},j},
\\[8pt]
\displaystyle
E_{\mathrm{e},i}-E_{\mathrm{e},j}
=
K_1(P_{\mathrm{e},i}-P_{\mathrm{e},j})
+
K_2(S_{\mathrm{e},i}-S_{\mathrm{e},j})
\end{cases}
$$

全局偏差为：

$$
\begin{cases}
\displaystyle
\Delta Q
=
Q_{\mathrm{wf}}^{\mathrm{ref}}
-
\sum_{k=1}^{N}Q_{\mathrm{d},k},\\[8pt]
\displaystyle
\Delta P
=
P_{\mathrm{wf}}^{\mathrm{ref}}
-\sum_{k=1}^{N}
\left(
P_{\mathrm{d},k}+P_{\mathrm{e},k}
\right)
\end{cases}
$$

其中，$P_{\mathrm{d},k}$ 是 DFIG 在 MPPT 作用下的有功输出，不是本文通信一致性控制直接调节的状态变量；在偏差变量建模中，它可作为外部运行点或扰动项处理。

将上述变量展开后，无功参考输入可以写为：

$$
\dot Q_{\mathrm{d},i}^{\mathrm{ref}}
=
-c_1
\sum_{j\in\vartheta_i}
\left(
\frac{1}{A_{\mathrm{d},i}}Q_{\mathrm{d},i}
-
\frac{1}{A_{\mathrm{d},j}}Q_{\mathrm{d},j}
\right)
-
c_0^{\mathrm{Q}}M_{i0}
\sum_{k=1}^{N}Q_{\mathrm{d},k}
+
c_0^{\mathrm{Q}}M_{i0}Q_{\mathrm{wf}}^{\mathrm{ref}}
$$

有功参考输入可以写为：

$$
\dot P_{\mathrm{e},i}^{\mathrm{ref}}
=
-c_2
\sum_{j\in\vartheta_i}
\left[
K_1(P_{\mathrm{e},i}-P_{\mathrm{e},j})
+
K_2(S_{\mathrm{e},i}-S_{\mathrm{e},j})
\right]
-
c_0^{\mathrm{P}}M_{i0}
\sum_{k=1}^{N}P_{\mathrm{e},k}
+
c_0^{\mathrm{P}}M_{i0}
\left(
P_{\mathrm{wf}}^{\mathrm{ref}}
-
\sum_{k=1}^{N}P_{\mathrm{d},k}
\right)
$$

由此可见，$\dot Q_{\mathrm{d},i}^{\mathrm{ref}}$ 与全场 $Q_{\mathrm{d}}$ 线性相关，$\dot P_{\mathrm{e},i}^{\mathrm{ref}}$ 与全场 $P_{\mathrm{e}}$、$S_{\mathrm{e}}$ 线性相关。若忽略外部调度指令和 MPPT 有功项，或在工作点附近写成偏差变量，常值项可吸收到平衡点中，于是全场输入可写成：

$$
u=Cx
$$

或者结合论文式 (10) 的符号约定写成：

$$
\dot x
=Ax-Bu
=(A-BC)x
$$

所以，$C$ 或 $C_{ij}$ 的作用是从全场状态中提取 $Q_{\mathrm{d}}$、$P_{\mathrm{e}}$、$S_{\mathrm{e}}$，并按通信拓扑、控制增益、$A_{\mathrm{d},i}$、$K_1,K_2$、leader 标记以及全局偏差反馈进行加权组合。它的本质是通信拓扑和一致性协议形成的状态反馈映射，也是后文写出 $C=\mathcal{H}(M_{\mathrm{c}})$、进而分析 $A_{\mathrm{cl}}=A-BC$ 的关键。

---

#### 13.7.6 当前理解小结

附录 A 的矩阵可以按如下方式理解：

$$
A_i
=
\begin{bmatrix}
A_{i,1}&0\\
0&A_{i,2}
\end{bmatrix}
$$

其中，$A_{i,1}$ 来自 DFIG 无功功率环，$A_{i,2}$ 来自 ES 有功功率环。

$B_i$ 描述上层参考输入 $\dot Q_{\mathrm{d},i}^{\mathrm{ref}}$、$\dot P_{\mathrm{e},i}^{\mathrm{ref}}$ 如何通过 PI 积分项和比例项注入本机状态；$C_{ij}$ 来自一致性控制律，描述第 $i$ 个节点如何根据邻居状态和全局功率偏差调整自身参考变化率。

因此，附录 A 不是凭空给出矩阵，而是把 Fig. 2 的 DFIG 无功环、Fig. 3 的 ES 有功环，以及基于 $E_{\mathrm{d},i}$ 和 $E_{\mathrm{e},i}$ 的一致性控制律统一整理成状态空间形式，为后续全场闭环矩阵 $A_{\mathrm{cl}}=A-BC$ 做准备。

### 13.8 当前理解小结

本节的核心可以概括为一条建模链条：

$$
\text{状态变量选取}
\rightarrow
\text{单台 WT 矩阵 }A_i,B_i
\rightarrow
\text{一致性反馈 }C_{ij}
\rightarrow
\text{全场闭环模型}.
$$

状态变量的选取原则是：凡是具有动态记忆、参与反馈控制或进入一致性变量的量，都需要作为状态。因此，$Q_{\mathrm{d},\mathrm{int},i}$ 和 $P_{\mathrm{e},\mathrm{int},i}$ 作为 PI 积分状态必须进入 $x_i$；它们不是初始值，而是误差累计量。

单台 WT 的底层动态由两部分组成：DFIG 无功通道和 ES 有功通道。前者包含 $Q_{\mathrm{d},i}$、$Q_{\mathrm{d},\mathrm{int},i}$、$i_{\mathrm{dr},i}$；后者包含 $P_{\mathrm{e},i}$、$P_{\mathrm{e},\mathrm{int},i}$、$i_{\mathrm{L},i}$、$S_{\mathrm{e},i}$。附录 A 中的 $A_i$ 和 $B_i$ 就是按这个状态顺序，把 Fig. 2、Fig. 3 的一阶环节、PI 积分状态和参考输入项整理出来。

上层一致性控制律则通过 $E_{\mathrm{d},i}$ 和 $E_{\mathrm{e},i}$ 生成参考变化率，并进一步整理成 $C_{ij}$。后续把所有 WT 的状态堆叠后，通信拓扑就通过 $C=\mathcal{H}(M_{\mathrm{c}})$ 进入全场闭环矩阵 $A_{\mathrm{cl}}=A-BC$。这也是后续能够讨论通信拓扑、闭环收敛速度和优化指标之间关系的基础。

## 14. 图论基础：本文真正用到的几个量

本文的图论部分只需要抓住通信网络的矩阵表达。通信图记为：

$$
G_{\mathrm{c}}=(V_{\mathrm{c}},E_{\mathrm{c}})
$$

其中每个节点对应一台 WT/ES 控制代理，边表示两个代理之间存在通信链路。本文默认通信图是无向、无权、无自环图。

---

### 14.1 邻接矩阵、度矩阵和拉普拉斯矩阵

通信邻接矩阵记为 $M_{\mathrm{c}}=[M_{\mathrm{c},ij}]$。对本文的无向无权图，有：

$$
M_{\mathrm{c},ij}
=
\begin{cases}
1, & i\text{ 与 }j\text{ 之间有通信边}\\
0, & i\text{ 与 }j\text{ 之间无通信边}
\end{cases},
\quad
M_{\mathrm{c}}=M_{\mathrm{c}}^{\mathrm{T}},
\quad
M_{\mathrm{c},ii}=0.
$$

度矩阵 $\Lambda_{\mathrm{c}}$ 是对角矩阵，其对角元素为节点度数：

$$
\Lambda_{\mathrm{c},ii}
=
\sum_{j=1}^{N}
M_{\mathrm{c},ij},
\quad
\Lambda_{\mathrm{c}}
=
\operatorname{diag}
\left(
\Lambda_{\mathrm{c},11},
\cdots,
\Lambda_{\mathrm{c},NN}
\right).
$$

通信拉普拉斯矩阵定义为：

$$
L_{\mathrm{c}}
=
\Lambda_{\mathrm{c}}-M_{\mathrm{c}}.
$$

也就是说，$M_{\mathrm{c}}$ 记录“谁和谁通信”，$\Lambda_{\mathrm{c}}$ 记录“每个节点有多少邻居”，$L_{\mathrm{c}}$ 则把通信拓扑转化为适合一致性控制分析的矩阵。

---

### 14.2 拉普拉斯矩阵的两个关键性质

对无向图，$M_{\mathrm{c}}=M_{\mathrm{c}}^{\mathrm{T}}$，而 $\Lambda_{\mathrm{c}}$ 是实对角矩阵，所以：

$$
L_{\mathrm{c}}^{\mathrm{T}}
=
\left(
\Lambda_{\mathrm{c}}-M_{\mathrm{c}}
\right)^{\mathrm{T}}
=
\Lambda_{\mathrm{c}}-M_{\mathrm{c}}
=
L_{\mathrm{c}}.
$$

因此 $L_{\mathrm{c}}$ 是实对称矩阵。半正定性可以由二次型看出。对任意 $x=[x_1,\cdots,x_N]^{\mathrm{T}}$，有：

$$
\begin{aligned}
x^{\mathrm{T}}L_{\mathrm{c}}x
&=
x^{\mathrm{T}}
\left(
\Lambda_{\mathrm{c}}-M_{\mathrm{c}}
\right)
x\\
&=
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}x_i^2
-
\sum_{i=1}^{N}
\sum_{j=1}^{N}
M_{\mathrm{c},ij}x_ix_j.
\end{aligned}
$$

由于 $\Lambda_{\mathrm{c},ii}=\sum_{j=1}^{N}M_{\mathrm{c},ij}$，且 $M_{\mathrm{c}}$ 对称，上式可整理为：

$$
x^{\mathrm{T}}L_{\mathrm{c}}x
=
\frac{1}{2}
\sum_{i=1}^{N}
\sum_{j=1}^{N}
M_{\mathrm{c},ij}
\left(
x_i-x_j
\right)^2
\geq 0.
$$

所以 $L_{\mathrm{c}}$ 是半正定矩阵。这个式子也给出直观含义：拉普拉斯二次型惩罚相邻节点之间的差异。若两个相连节点状态差越大，则该项贡献越大。

另一个关键性质是：

$$
L_{\mathrm{c}}1_N=0.
$$

这是因为 $L_{\mathrm{c}}$ 的每一行元素之和为零：

$$
\Lambda_{\mathrm{c},ii}
-
\sum_{j=1}^{N}
M_{\mathrm{c},ij}
=
0.
$$

因此，$0$ 总是 $L_{\mathrm{c}}$ 的一个特征值，特征向量为全 1 向量 $1_N$。在一致性控制中，$1_N$ 方向对应所有节点状态相同的“一致状态”，所以这个零特征值不是异常，而是拉普拉斯矩阵固有的结构。

---

### 14.3 代数连通度 $\lambda_2(L_{\mathrm{c}})$

由于 $L_{\mathrm{c}}$ 是实对称半正定矩阵，其特征值可以排列为：

$$
0
=
\lambda_1
\left(
L_{\mathrm{c}}
\right)
\leq
\lambda_2
\left(
L_{\mathrm{c}}
\right)
\leq
\cdots
\leq
\lambda_N
\left(
L_{\mathrm{c}}
\right).
$$

第二小特征值 $\lambda_2(L_{\mathrm{c}})$ 称为代数连通度，也称 Fiedler 值。它最重要的判据是：

$$
\lambda_2
\left(
L_{\mathrm{c}}
\right)
>
0
\Longleftrightarrow
G_{\mathrm{c}}\text{ 连通}.
$$

这个判据可以从上一节的二次型理解。若：

$$
x^{\mathrm{T}}L_{\mathrm{c}}x=0,
$$

则必须对每一条通信边都有：

$$
x_i=x_j.
$$

如果通信图连通，那么任意两个节点之间都可以通过一条路径连接，上式会沿路径传递，最终得到：

$$
x_1=x_2=\cdots=x_N.
$$

因此，连通图中 $L_{\mathrm{c}}$ 的零空间只有全 1 方向，即零特征值只有一个，所以下一个特征值满足 $\lambda_2(L_{\mathrm{c}})>0$。

反过来，如果图不连通，每个连通分量都可以各自取一个常数值，而不同分量之间不需要相等。这样会产生多个彼此独立的零特征向量，所以 $\lambda_2(L_{\mathrm{c}})=0$。更一般地，零特征值的重数等于连通分量个数。

在一致性系统：

$$
\dot x=-L_{\mathrm{c}}x
$$

中，$\lambda_1=0$ 对应最终一致方向，不代表误差衰减；真正影响最慢一致性误差衰减速度的是 $\lambda_2(L_{\mathrm{c}})$。因此可以粗略理解为：$\lambda_2(L_{\mathrm{c}})$ 越大，通信网络越紧密，信息扩散和一致性收敛通常越快。

---

### 14.4 本文中这些量的作用

后续优化中，这几个图论量分别承担不同角色：

| 量 | 在本文中的作用 |
|---|---|
| $M_{\mathrm{c}}$ | 决定具体通信边，也是主要拓扑决策变量 |
| $\Lambda_{\mathrm{c}}$ | 表示节点度数，进入通信稀疏性和节点度均衡目标 |
| $L_{\mathrm{c}}$ | 表示一致性反馈中的邻居差异结构 |
| $\lambda_2(L_{\mathrm{c}})$ | 判断通信图是否连通，并作为连通性/收敛性的谱指标 |

因此，本文后续从通信拓扑到闭环控制性能的核心链条是：

$$
M_{\mathrm{c}}
\rightarrow
\Lambda_{\mathrm{c}},L_{\mathrm{c}}
\rightarrow
C=\mathcal{H}
\left(
M_{\mathrm{c}}
\right)
\rightarrow
A_{\mathrm{cl}}
=
A-BC.
$$

第 14 节只需要记住这一点：图论不是独立主题，而是为了把通信网络写成矩阵，并让它进入后续的闭环系统和优化问题。

## 15. 从单台 WT 模型到全场闭环模型：式 (9)、式 (10) 与附录 B 的理解

Section III-B 开始，论文将前文建立的单台 WT 状态空间模型进一步扩展到整个风电场。这里最容易混淆的是：式 (9) 中单台 WT 模型写成加号形式，而式 (10) 中全场模型写成减号形式。

论文式 (9) 为：

$$
\dot x_i=A_ix_i+B_iu_i
$$

$$
u_i=-\sum_{j\in\vartheta_i}C_{ij}x_j
$$

而式 (10) 写成：

$$
\dot x=Ax-Bu=(A-BC)x=A_{\mathrm{cl}}x
$$

$$
C=\mathcal H(M_{\mathrm{c}})
$$

这两个式子并不矛盾，差别主要来自对输入符号的定义方式不同。

---

### 15.1 为什么式 (9) 是加号，而式 (10) 是减号

式 (9) 的第一行：

$$
\dot x_i=A_ix_i+B_iu_i
$$

是标准状态空间输入形式，表示输入 $u_i$ 通过输入矩阵 $B_i$ 作用于单台 WT 的状态动态。

但是式 (9) 的第二行又规定：

$$
u_i=-\sum_{j\in\vartheta_i}C_{ij}x_j
$$

也就是说，$u_i$ 本身已经是一个带负号的一致性反馈输入。将第二行代入第一行，可得：

$$
\dot x_i
=A_i x_i-B_i \sum_{j\in\vartheta_i}C_{ij}x_j
$$

因此，虽然式 (9) 第一行表面上是加号，但由于 $u_i$ 自身包含负反馈符号，实际闭环效果仍然是：

$$
A_i-B_iC_i
$$

这一点和普通负反馈系统是类似的。可以有两种等价写法。

第一种写法是把负号放进输入定义中：

$$
\dot x=A x+B u_{\mathrm{actual}}
$$

$$
u_{\mathrm{actual}}=-Cx
$$

代入后：

$$
\dot x=(A-BC)x
$$

第二种写法是把输入定义为正的反馈量，再在系统方程中显式写出负号：

$$
u=Cx
$$

$$
\dot x=Ax-Bu
$$

代入后同样得到：

$$
\dot x=(A-BC)x
$$

式 (9) 更接近第一种写法，式 (10) 更接近第二种写法。因此，式 (9) 与式 (10) 的符号差异不是物理含义改变，而是负反馈符号放置位置不同。

可以理解为：

$$
\boxed{
\text{式 (9)：输入通道写成 }+B_iu_i,\text{ 但 }u_i\text{ 已经包含负号}
}
$$

$$
\boxed{
\text{式 (10)：重新定义 }u=Cx,\text{ 因此负号显式写在 }-Bu\text{ 中}
}
$$

最终二者都对应：

$$
\boxed{
\dot x=(A-BC)x
}
$$

---

### 15.2 全场状态变量的堆叠

在式 (10) 中，论文将所有 WT 的状态变量堆叠为全场状态向量：

$$
x=
\begin{bmatrix}
Q_{\mathrm{d}}^{\mathrm{T}}&
Q_{\mathrm{d},\mathrm{int}}^{\mathrm{T}}&
i_{\mathrm{dr}}^{\mathrm{T}}&
P_{\mathrm{e}}^{\mathrm{T}}&
P_{\mathrm{e},\mathrm{int}}^{\mathrm{T}}&
i_{\mathrm{L}}^{\mathrm{T}}&
S_{\mathrm{e}}^{\mathrm{T}}
\end{bmatrix}^{\mathrm{T}}
$$

其中：

$$
Q_{\mathrm{d}}=
\begin{bmatrix}
Q_{\mathrm{d},1}&Q_{\mathrm{d},2}&\cdots&Q_{\mathrm{d},N}
\end{bmatrix}^{\mathrm{T}}
$$

$$
P_{\mathrm{e}}=
\begin{bmatrix}
P_{\mathrm{e},1}&P_{\mathrm{e},2}&\cdots&P_{\mathrm{e},N}
\end{bmatrix}^{\mathrm{T}}
$$

$$
S_{\mathrm{e}}=
\begin{bmatrix}
S_{\mathrm{e},1}&S_{\mathrm{e},2}&\cdots&S_{\mathrm{e},N}
\end{bmatrix}^{\mathrm{T}}
$$

全场输入变量为：

$$
u=
\begin{bmatrix}
(\dot Q_{\mathrm{d}}^{\mathrm{ref}})^{\mathrm{T}}&
(\dot P_{\mathrm{e}}^{\mathrm{ref}})^{\mathrm{T}}
\end{bmatrix}^{\mathrm{T}}
$$

其中：

$$
\dot Q_{\mathrm{d}}^{\mathrm{ref}}
=

\begin{bmatrix}
\dot Q_{\mathrm{d},1}^{\mathrm{ref}}&
\dot Q_{\mathrm{d},2}^{\mathrm{ref}}&
\cdots&
\dot Q_{\mathrm{d},N}^{\mathrm{ref}}
\end{bmatrix}^{\mathrm{T}}
$$

$$
\dot P_{\mathrm{e}}^{\mathrm{ref}}
=

\begin{bmatrix}
\dot P_{\mathrm{e},1}^{\mathrm{ref}}&
\dot P_{\mathrm{e},2}^{\mathrm{ref}}&
\cdots&
\dot P_{\mathrm{e},N}^{\mathrm{ref}}
\end{bmatrix}^{\mathrm{T}}
$$

矩阵 $A$ 和 $B$ 是由所有单台 WT 的 $A_i$ 和 $B_i$ 堆叠得到的全场矩阵。若各 WT 参数一致，则它们具有重复的块结构；若参数不同，则各个块可以不同。

---

### 15.3 附录 B 的核心目的

附录 B 的核心任务是说明：上层一致性控制生成的参考输入 $u$ 可以写成状态变量 $x$ 的线性组合，即：

$$
u=Cx
$$

更具体地说，需要说明：

$$
\dot Q_{\mathrm{d}}^{\mathrm{ref}}
$$

可以由 $Q_{\mathrm{d}}$ 线性表示，而：

$$
\dot P_{\mathrm{e}}^{\mathrm{ref}}
$$

可以由 $P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 线性表示。

由于 $Q_{\mathrm{d}}$、$P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 都是全场状态向量 $x$ 的组成部分，因此可以进一步得到：

$$
u=Cx
$$

这一步是式 (10) 成立的关键。

---

### 15.4 无功参考变化率的展开

DFIG 无功一致性变量为：

$$
E_{\mathrm{d},i}=
\frac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}}
$$

将所有机组写成向量形式，可定义：

$$
E_{\mathrm{d}}=
D_{\mathrm{A}}Q_{\mathrm{d}}
$$

其中：

$$
D_{\mathrm{A}}=
\mathrm{diag}
\left(
\frac{1}{A_{\mathrm{d},1}},
\frac{1}{A_{\mathrm{d},2}},
\cdots,
\frac{1}{A_{\mathrm{d},N}}
\right)
$$

一致性控制中的邻居差异项可以用通信拉普拉斯矩阵 $L_{\mathrm{c}}$ 表示。于是无功参考变化率可以写成：

$$
\dot Q_{\mathrm{d}}^{\mathrm{ref}}
=
-c_1L_{\mathrm{c}}E_{\mathrm{d}}
+
c_0^{\mathrm{Q}}\Gamma_{\mathrm{c},0}\Delta Q
$$

其中，$\Gamma_{\mathrm{c},0}$ 表示 leader 节点标记向量，$c_1$ 是无功一致性控制增益，$c_0^{\mathrm{Q}}$ 是无功全局偏差反馈增益。

风电场无功偏差为：

$$
\Delta Q
=
Q_{\mathrm{wf}}^{\mathrm{ref}}
-1_N^{\mathrm{T}}Q_{\mathrm{d}}
$$
>注：$1_N$ 表示长度为 $N$ 的全 1 列向量，即 $1_N=[1,1,\cdots,1]^{\mathrm{T}}$；因此 $1_N^{\mathrm{T}}$ 是全 1 行向量。若 $Q_{\mathrm{d}}=[Q_{\mathrm{d},1},Q_{\mathrm{d},2},\cdots,Q_{\mathrm{d},N}]^{\mathrm{T}}$，则 $1_N^{\mathrm{T}}Q_{\mathrm{d}}=\sum_{i=1}^{N}Q_{\mathrm{d},i}$，表示对所有节点对应变量求和。同理，$1_N^{\mathrm{T}}P_{\mathrm{e}}$ 表示所有 ES 有功功率之和，$1_N^{\mathrm{T}}P_{\mathrm{d}}$ 表示所有 DFIG 有功功率之和。

代入 $E_{\mathrm{d}}=D_{\mathrm{A}}Q_{\mathrm{d}}$ 和 $\Delta Q$，可得：

$$
\dot Q_{\mathrm{d}}^{\mathrm{ref}}
=

-c_1L_{\mathrm{c}}D_{\mathrm{A}}Q_{\mathrm{d}}
+
c_0^{\mathrm{Q}}\Gamma_{\mathrm{c},0}
\left(
Q_{\mathrm{wf}}^{\mathrm{ref}}
-

1_N^{\mathrm{T}}Q_{\mathrm{d}}
\right)
$$

整理为：

$$
\dot Q_{\mathrm{d}}^{\mathrm{ref}}
=

-\left(
c_1L_{\mathrm{c}}D_{\mathrm{A}}
+
c_0^{\mathrm{Q}}\Gamma_{\mathrm{c},0}1_N^{\mathrm{T}}
\right)Q_{\mathrm{d}}
+
c_0^{\mathrm{Q}}\Gamma_{\mathrm{c},0}Q_{\mathrm{wf}}^{\mathrm{ref}}
$$

该式说明，无功参考变化率 $\dot Q_{\mathrm{d}}^{\mathrm{ref}}$ 与全场无功状态 $Q_{\mathrm{d}}$ 线性相关，同时包含由无功调度指令 $Q_{\mathrm{wf}}^{\mathrm{ref}}$ 引起的外部输入项。

如果在某个工作点附近采用偏差变量建模，或只分析闭环收敛特性，则外部常值项可被吸收到平衡点中。此时主要关注的是：

$$
\dot Q_{\mathrm{d}}^{\mathrm{ref}}
\sim
-\left(
c_1L_{\mathrm{c}}D_{\mathrm{A}}
+
c_0^{\mathrm{Q}}\Gamma_{\mathrm{c},0}1_N^{\mathrm{T}}
\right)Q_{\mathrm{d}}
$$

因此，$\dot Q_{\mathrm{d}}^{\mathrm{ref}}$ 可以由状态变量 $Q_{\mathrm{d}}$ 线性表示。

---

### 15.5 有功参考变化率的展开

ES 一致性变量为：

$$
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}
$$

写成向量形式为：

$$
E_{\mathrm{e}}=K_1P_{\mathrm{e}}+K_2S_{\mathrm{e}}
$$

ES 有功参考变化率可以写成：

$$
\dot P_{\mathrm{e}}^{\mathrm{ref}}
=
-c_2L_{\mathrm{c}}E_{\mathrm{e}}
+
c_0^{\mathrm{P}}\Gamma_{\mathrm{c},0}\Delta P
$$

其中，$c_2$ 是 ES 有功一致性控制增益，$c_0^{\mathrm{P}}$ 是有功全局偏差反馈增益。

风电场有功偏差为：

$$
\Delta P
=
P_{\mathrm{wf}}^{\mathrm{ref}}-1_N^{\mathrm{T}}(P_{\mathrm{d}}+P_{\mathrm{e}})
$$

也就是：

$$
\Delta P
=
P_{\mathrm{wf}}^{\mathrm{ref}}-1_N^{\mathrm{T}}P_{\mathrm{d}}-1_N^{\mathrm{T}}P_{\mathrm{e}}
$$

将 $E_{\mathrm{e}}=K_1P_{\mathrm{e}}+K_2S_{\mathrm{e}}$ 和 $\Delta P$ 代入，可得：

$$
\dot P_{\mathrm{e}}^{\mathrm{ref}}
=
-c_2L_{\mathrm{c}}
\left(
K_1P_{\mathrm{e}}+K_2S_{\mathrm{e}}
\right)
+
c_0^{\mathrm{P}}\Gamma_{\mathrm{c},0}
\left(
P_{\mathrm{wf}}^{\mathrm{ref}}
-1_N^{\mathrm{T}}P_{\mathrm{d}}-1_N^{\mathrm{T}}P_{\mathrm{e}}
\right)
$$

整理为：

$$
\dot P_{\mathrm{e}}^{\mathrm{ref}}
=

-\left(
c_2K_1L_{\mathrm{c}}
+
c_0^{\mathrm{P}}\Gamma_{\mathrm{c},0}1_N^{\mathrm{T}}
\right)P_{\mathrm{e}}
-

c_2K_2L_{\mathrm{c}}S_{\mathrm{e}}
+
c_0^{\mathrm{P}}\Gamma_{\mathrm{c},0}
\left(
P_{\mathrm{wf}}^{\mathrm{ref}}
-

1_N^{\mathrm{T}}P_{\mathrm{d}}
\right)
$$

该式说明，有功参考变化率 $\dot P_{\mathrm{e}}^{\mathrm{ref}}$ 与 $P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 线性相关，同时包含由风电场有功指令和 DFIG MPPT 有功输出形成的外部项：

$$
c_0^{\mathrm{P}}\Gamma_{\mathrm{c},0}
\left(
P_{\mathrm{wf}}^{\mathrm{ref}}
-

1_N^{\mathrm{T}}P_{\mathrm{d}}
\right)
$$

其中，$P_{\mathrm{d}}$ 不是本文通信一致性控制直接调节的状态，而是由风速和 MPPT 决定的外部运行量。因此，在闭环收敛分析或偏差变量建模中，该项可以作为外部输入或工作点项处理。

由此可见，$\dot P_{\mathrm{e}}^{\mathrm{ref}}$ 可以由状态变量 $P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 线性表示。

---

### 15.6 从附录 B 到 $u=Cx$

根据前面的展开：

$$
\dot Q_{\mathrm{d}}^{\mathrm{ref}}
$$

可以由 $Q_{\mathrm{d}}$ 线性表示；

$$
\dot P_{\mathrm{e}}^{\mathrm{ref}}
$$

可以由 $P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 线性表示。

而 $Q_{\mathrm{d}}$、$P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 都属于全场状态向量：

$$
x=
\begin{bmatrix}
Q_{\mathrm{d}}^{\mathrm{T}}&
Q_{\mathrm{d},\mathrm{int}}^{\mathrm{T}}&
i_{\mathrm{dr}}^{\mathrm{T}}&
P_{\mathrm{e}}^{\mathrm{T}}&
P_{\mathrm{e},\mathrm{int}}^{\mathrm{T}}&
i_{\mathrm{L}}^{\mathrm{T}}&
S_{\mathrm{e}}^{\mathrm{T}}
\end{bmatrix}^{\mathrm{T}}
$$

因此，可以将全场输入统一写成：

$$
u=Cx
$$

其中，$C$ 的非零列主要对应 $Q_{\mathrm{d}}$、$P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 这些直接参与一致性控制的状态；对应 $Q_{\mathrm{d},\mathrm{int}}$、$i_{\mathrm{dr}}$、$P_{\mathrm{e},\mathrm{int}}$、$i_{\mathrm{L}}$ 的列通常为零。

从物理意义看：

$$
C
$$

并不是单台设备的底层物理模型矩阵，而是由通信拓扑和一致性控制律共同形成的反馈矩阵。它把以下信息统一吸收进矩阵元素：

1. 通信拉普拉斯矩阵 $L_{\mathrm{c}}$；
2. DFIG 无功可调空间 $A_{\mathrm{d},i}$；
3. ES 一致性权重 $K_1$ 和 $K_2$；
4. leader 节点标记 $\Gamma_{\mathrm{c},0}$；
5. 一致性控制增益 $c_1$、$c_2$；
6. 全局偏差反馈增益 $c_0^{\mathrm{Q}}$、$c_0^{\mathrm{P}}$。

---

### 15.7 为什么 $C=\mathcal H(M_{\mathrm{c}})$

论文进一步写道：

$$
C=\mathcal H(M_{\mathrm{c}})
$$

其含义是：反馈矩阵 $C$ 由通信邻接矩阵 $M_{\mathrm{c}}$ 通过某种线性映射得到。

原因是通信拓扑首先决定邻接矩阵 $M_{\mathrm{c}}$，再由 $M_{\mathrm{c}}$ 得到度矩阵 $\Lambda_{\mathrm{c}}$ 和拉普拉斯矩阵：

$$
L_{\mathrm{c}}=\Lambda_{\mathrm{c}}-M_{\mathrm{c}}
$$

而在附录 B 的推导中，$\dot Q_{\mathrm{d}}^{\mathrm{ref}}$ 和 $\dot P_{\mathrm{e}}^{\mathrm{ref}}$ 都显式含有 $L_{\mathrm{c}}$。因此，通信拓扑改变时，$L_{\mathrm{c}}$ 改变；$L_{\mathrm{c}}$ 改变时，输入反馈矩阵 $C$ 改变；最终闭环矩阵：

$$
A_{\mathrm{cl}}=A-BC
$$

也会改变。

这一逻辑链条可以写成：

$$
M_{\mathrm{c}}
\rightarrow
L_{\mathrm{c}}
\rightarrow
C
\rightarrow
A_{\mathrm{cl}}=A-BC
\rightarrow
\text{闭环收敛性能}
$$

所以，Section III-B 之后的通信网络优化，本质上就是通过选择合适的通信邻接矩阵 $M_{\mathrm{c}}$，改变反馈矩阵 $C$，从而改变闭环系统矩阵 $A_{\mathrm{cl}}$ 的特征值和收敛速度。

---

### 15.8 当前理解小结

式 (9) 和式 (10) 的关系可以概括为：

$$
\boxed{
\text{式 (9) 是单台 WT 的本地状态空间模型}
}
$$

$$
\boxed{
\text{式 (10) 是所有 WT 堆叠后的全场闭环模型}
}
$$

式 (9) 中：

$$
\dot x_i=A_ix_i+B_iu_i
$$

只是标准输入通道写法；而：

$$
u_i=-\sum_{j\in\vartheta_i}C_{ij}x_j
$$

说明输入本身是负反馈一致性控制。因此代入后仍然得到负反馈闭环。

式 (10) 中，论文将输入重新整理为：

$$
u=Cx
$$

所以全场状态方程写成：

$$
\dot x=Ax-Bu=(A-BC)x
$$

附录 B 的作用就是证明：

$$
u=
\begin{bmatrix}
(\dot Q_{\mathrm{d}}^{\mathrm{ref}})^{\mathrm{T}}&
(\dot P_{\mathrm{e}}^{\mathrm{ref}})^{\mathrm{T}}
\end{bmatrix}^{\mathrm{T}}
$$

可以由状态向量 $x$ 线性表示，从而得到：

$$
u=Cx
$$

进一步，由于 $C$ 由通信拉普拉斯矩阵 $L_{\mathrm{c}}$ 决定，而 $L_{\mathrm{c}}$ 又由通信邻接矩阵 $M_{\mathrm{c}}$ 决定，所以：

$$
C=\mathcal H(M_{\mathrm{c}})
$$

这就是论文能够把通信拓扑优化问题转化为闭环状态矩阵优化问题的关键。

## 16. 收敛速度目标的理解：闭环矩阵、矩阵指数与状态范数

Section III-B 中，论文提出通信网络优化需要同时考虑三个目标：控制收敛速度、通信稀疏性和通信生存性。其中第一个目标是提高控制收敛速度，以便在风速剧烈波动时更快地平滑风电场输出并跟踪调度指令。

由前文可知，整个风电场的闭环模型已经被写成：

$$
\dot{x}=A_{\mathrm{cl}}x
$$

其中：

$$
A_{\mathrm{cl}}=A-BC
$$

这里 $A_{\mathrm{cl}}$ 是整个风电场分布式控制系统的闭环状态矩阵。由于通信拓扑 $M_{\mathrm{c}}$ 会影响反馈矩阵 $C$，因此也会影响闭环矩阵 $A_{\mathrm{cl}}$。

---

### 16.1 线性矩阵微分方程的解

标量一阶线性微分方程：

$$
\dot{x}=ax
$$

的解为：

$$
x(t)=e^{a(t-t_0)}x_0
$$

矩阵状态方程：

$$
\dot{x}=A_{\mathrm{cl}}x
$$

可以看成它的推广。其解为：

$$
x(t)=e^{A_{\mathrm{cl}}(t-t_0)}x_0
$$

其中，$e^{A_{\mathrm{cl}}(t-t_0)}$ 称为矩阵指数。它描述了初始状态 $x_0$ 在闭环系统矩阵 $A_{\mathrm{cl}}$ 作用下随时间演化的过程。

如果 $A_{\mathrm{cl}}$ 可以对角化，即：

$$
A_{\mathrm{cl}}=V\Lambda V^{-1}
$$

则有：

$$
e^{A_{\mathrm{cl}}t}
=
Ve^{\Lambda t}V^{-1}
$$

其中：

$$
e^{\Lambda t}
=

\operatorname{diag}
\left(
e^{\lambda_1t},
e^{\lambda_2t},
\cdots,
e^{\lambda_nt}
\right)
$$

因此，闭环系统可以理解为由多个模态叠加而成，每个模态大致按照 $e^{\lambda_it}$ 的形式变化。

---

### 16.2 为什么特征值实部决定收敛速度

对于某个特征值 $\lambda_i$，若其实部满足：

$$
\operatorname{Re}(\lambda_i)<0
$$

则对应模态会随时间衰减；若：

$$
\operatorname{Re}(\lambda_i)>0
$$

则对应模态会随时间增长。

因此，闭环系统渐近稳定的基本条件是：

$$
\operatorname{Re}(\lambda_i(A_{\mathrm{cl}}))<0,\quad i=1,2,\cdots,n
$$

论文定义：

$$
k_{\max} = \operatorname{Re} \left\{ \lambda_{\max}(A_{\mathrm{cl}}) \right\}
$$

这里的 $k_{\max}$ 可以理解为 $A_{\mathrm{cl}}$ 所有特征值中最靠近虚轴的那个实部，也就是最慢衰减模态对应的实部。

如果：

$$
k_{\max}<0
$$

则所有模态最终都会衰减，系统渐近稳定。

如果进一步要求：

$$
k_{\max}<-v_{\mathrm{spe}}
$$

则表示所有特征值都位于复平面直线 $-v_{\mathrm{spe}}$ 的左侧，系统收敛速度至少满足给定指标 $v_{\mathrm{spe}}$。

因此，论文希望通过设计通信拓扑 $M_{\mathrm{c}}$ 来改变 $C$，进而改变：

$$
A_{\mathrm{cl}}=A-BC
$$

的特征值分布，使闭环系统具有更快的收敛速度。

---

### 16.3 状态向量的 2-范数含义

论文中使用了状态向量的 2-范数：

$$
\|x(t)\|_{\ell_2}
$$

它也常写作：

$$
\|x(t)\|_2
$$

对于向量：

$$
x=
\begin{bmatrix}
x_1&
x_2&
\cdots&
x_n
\end{bmatrix}^{\mathrm{T}}
$$

其 2-范数定义为：

$$
\|x\|_2
=

\sqrt{
x_1^2+x_2^2+\cdots+x_n^2
}
$$

在本文中，$x$ 包含风电场中所有 WT/ES 的状态，例如 $Q_{\mathrm{d}}$、$P_{\mathrm{e}}$、$S_{\mathrm{e}}$、PI 积分状态和电流状态等。因此，$\|x(t)\|_2$ 可以理解为整个闭环系统状态偏差的整体大小。

需要注意，不同状态量可能具有不同物理单位，因此普通 2-范数本身不一定直接对应某个物理能量。后文引入加权二次型 $x^{\mathrm{T}}Qx$，就是为了通过权重矩阵 $Q$ 对不同状态的重要性进行调整。

---

### 16.4 式 (11) 的直观含义

论文式 (11) 用状态范数说明闭环收敛速度与最右侧特征值实部的关系，可理解为：

$$
\|x(t)\|_{\ell_2}
\leq
e^{k_{\max}(t-t_0)}
\|x_0\|_{\ell_2},\quad t\geq t_0
$$

这个式子的直观含义是：闭环状态向量的整体大小由指数项控制，而指数项的衰减速度由 $k_{\max}$ 决定。

如果 $k_{\max}<0$，指数项随时间衰减；如果 $k_{\max}$ 更负，指数项衰减得更快。因此，式 (11) 的作用不是引入新的目标，而是把“闭环极点位置”和“状态衰减快慢”联系起来。

---

### 16.5 关于该范数估计的一个数学细节

从工程控制角度看，用最右侧特征值实部 $k_{\max}$ 描述收敛速度是很常见的做法。它主要刻画系统的渐近模态衰减速度。

但如果从严格矩阵分析角度看，状态 2-范数的瞬时增长或衰减还与矩阵 $A_{\mathrm{cl}}$ 的非正规性有关。如果 $A_{\mathrm{cl}}$ 的特征向量不正交，即矩阵具有明显非正规性，那么即使所有特征值实部都为负，状态 2-范数也可能在短时间内出现暂态放大。

更严格地估计状态范数时，常引入矩阵的对数范数，也称矩阵测度。对于 2-范数，对数范数为：

$$
\mu_2(A_{\mathrm{cl}})
=

\lambda_{\max}
\left(
\frac{
A_{\mathrm{cl}}+A_{\mathrm{cl}}^{\mathrm{T}}
}{2}
\right)
$$

它满足：

$$
\frac{\mathrm{d}}{\mathrm{d}t}\|x(t)\|_2
\leq
\mu_2(A_{\mathrm{cl}})
\|x(t)\|_2
$$

因此可以得到严格的范数上界：

$$
\|x(t)\|_2
\leq
e^{\mu_2(A_{\mathrm{cl}})(t-t_0)}
\|x_0\|_2
$$

一般而言：

$$
k_{\max}
\leq
\mu_2(A_{\mathrm{cl}})
$$

所以对数范数给出的范数衰减估计更直接、更保守；而 $k_{\max}$ 更常用于描述闭环极点位置和渐近收敛速度。

---

### 16.6 Lemma 1 的作用：把收敛速度要求转化为可优化指标

前文已经说明，收敛速度要求可以写成：

$$
k_{\max}<-v_{\mathrm{spe}}
$$

也就是希望闭环矩阵 $A_{\mathrm{cl}}$ 的所有特征值都位于复平面直线 $-v_{\mathrm{spe}}$ 的左侧。

但是，在通信拓扑优化中，直接把特征值位置作为约束并不方便。Lemma 1 的作用就是引入一个更容易放进优化问题的表达：用带指数权重的二次积分指标来反映该收敛速度要求。

因此，Lemma 1 可以理解为：

$$
\boxed{
\text{把“特征值位于 }-v_{\mathrm{spe}}\text{ 左侧”的收敛速度要求，转化为一个可优化的积分型二次指标。}
}
$$

---

### 16.7 为什么构造带指数权重的积分指标

如果不指定收敛速度，常见的二次积分指标为：

$$
\int_0^{\infty}
\left[
x^{\mathrm{T}}(t)Qx(t)
+
u^{\mathrm{T}}(t)Ru(t)
\right]\,\mathrm{d}t
$$

它关心状态偏差和控制输入的累计代价，但本身不直接规定状态必须以多快速度衰减。

为了把收敛速度要求也放进指标中，论文式 (12) 在二次项前加入指数权重：

$$
\int_0^{\infty}
e^{2v_{\mathrm{spe}}t}
\left[
x^{\mathrm{T}}(t)Qx(t)
+
u^{\mathrm{T}}(t)Ru(t)
\right]\,\mathrm{d}t
$$

该式可以看作带收敛速度要求的 LQR 型二次性能指标 [28，Anderson and Moore, 1971, “Linear Optimal Control”]。这里的 “LQR 型” 只是指目标函数具有“状态二次项 + 输入二次项”的结构；本文并不是直接求解标准 LQR 最优反馈，而是用这种指标评价通信拓扑诱导出的闭环性能。

> 注：标准 LQR 问题通常是：给定线性系统 $\dot{x}=Ax-Bu$，在反馈矩阵 $C$ 可以自由选择的情况下，最小化 $\int_0^\infty (x^{\mathrm{T}}Qx+u^{\mathrm{T}}Ru)\,\mathrm{d}t$，并由 Riccati 方程求出最优反馈 $u=C_{\mathrm{LQR}}x$。本文与它不同：反馈矩阵 $C$ 不是自由连续变量，而是由通信拓扑 $M_{\mathrm{c}}$ 决定。因此这里说 “LQR 型”，重点是借用这种二次性能指标结构，而不是说论文在直接做标准 LQR 控制器设计。

指数权重的作用是放大后期误差：如果系统状态衰减不够快，即使最终趋于零，乘上 $e^{2v_{\mathrm{spe}}t}$ 后，积分值仍可能变大甚至发散。

假设某个状态模态近似满足：

$$
x(t)\sim e^{\lambda t}
$$

则二次型大致满足：

$$
x^{\mathrm{T}}(t)Qx(t)\sim e^{2\operatorname{Re}(\lambda)t}
$$

乘上指数权重后得到：

$$
e^{2v_{\mathrm{spe}}t}
e^{2\operatorname{Re}(\lambda)t}
=

e^{2\left(\operatorname{Re}(\lambda)+v_{\mathrm{spe}}\right)t}
$$

若希望该项在 $[0,\infty)$ 上可积，则需要：

$$
\operatorname{Re}(\lambda)+v_{\mathrm{spe}}<0
$$

也就是：

$$
\operatorname{Re}(\lambda)<-v_{\mathrm{spe}}
$$

这正好对应论文希望满足的特征值位置要求。因此，指数权重的意义可以概括为：

$$
\boxed{
e^{2v_{\mathrm{spe}}t}
\text{ 用来惩罚衰减不够快的状态和输入，从而强制闭环系统具有至少 }v_{\mathrm{spe}}\text{ 的指数收敛速度。}
}
$$

---

### 16.8 为什么指数是 $2v_{\mathrm{spe}}$

这里的系数 $2$ 来自二次型。若希望状态本身至少按照：

$$
e^{-v_{\mathrm{spe}}t}
$$

衰减，那么状态二次型大致按照：

$$
e^{-2v_{\mathrm{spe}}t}
$$

衰减。

因此，指标中使用 $e^{2v_{\mathrm{spe}}t}$，是为了和“状态平方”产生的二倍指数阶数匹配，用来检验状态是否衰减得快于 $e^{-v_{\mathrm{spe}}t}$。等价地，定义加权状态：

$$
z(t)=e^{v_{\mathrm{spe}}t}x(t)
$$

则有：

$$
z^{\mathrm{T}}Qz
=

e^{2v_{\mathrm{spe}}t}x^{\mathrm{T}}Qx
$$

所以，式 (12) 中的指数权重也可以理解为：不是直接对 $x(t)$ 做普通二次积分，而是对加权后的状态 $z(t)$ 做普通二次积分。

---

### 16.9 指数权重与矩阵平移的关系

继续使用加权状态：

$$
z(t)=e^{v_{\mathrm{spe}}t}x(t)
$$

由闭环系统：

$$
\dot{x}=A_{\mathrm{cl}}x
$$

可得：

$$
\dot{z}
=
v_{\mathrm{spe}}e^{v_{\mathrm{spe}}t}x
+
e^{v_{\mathrm{spe}}t}\dot{x}
$$

代入 $\dot{x}=A_{\mathrm{cl}}x$，得到：

$$
\dot{z}
=
\left(A_{\mathrm{cl}}+v_{\mathrm{spe}}I\right)z
$$

因此，要求加权状态 $z(t)$ 稳定，相当于要求：

$$
A_{\mathrm{cl}}+v_{\mathrm{spe}}I
$$

是 Hurwitz 矩阵。

它的特征值为原闭环特征值整体右移 $v_{\mathrm{spe}}$，因此稳定条件为：

$$
\operatorname{Re}
\left(
\lambda_i(A_{\mathrm{cl}})+v_{\mathrm{spe}}
\right)<0
$$

也就是：

$$
\operatorname{Re}
\left(
\lambda_i(A_{\mathrm{cl}})
\right)<-v_{\mathrm{spe}}
$$

所以，指数加权和矩阵平移其实是同一件事的两种写法：

$$
\boxed{
\text{对积分加 }e^{2v_{\mathrm{spe}}t}
\Longleftrightarrow
\text{对闭环矩阵看 }A_{\mathrm{cl}}+v_{\mathrm{spe}}I
}
$$

---

### 16.10 为什么性能指标中同时包含状态项和输入项

式 (12) 中的积分同时包含两个二次代价项：

$$
x^{\mathrm{T}}(t)Qx(t)
$$

和：

$$
u^{\mathrm{T}}(t)Ru(t)
$$

其中，$x^{\mathrm{T}}Qx$ 是状态代价，用于惩罚闭环状态偏差。本文状态 $x$ 中包含 DFIG 无功功率、ES 有功功率、SOC、PI 积分状态和电流内环状态等，因此该项表达的是“状态不要长时间偏离平衡点”。

$u^{\mathrm{T}}Ru$ 是控制输入代价，用于惩罚控制动作过大。本文中：

$$
u=
\begin{bmatrix}
(\dot Q_{\mathrm{d}}^{\mathrm{ref}})^{\mathrm{T}}&
(\dot P_{\mathrm{e}}^{\mathrm{ref}})^{\mathrm{T}}
\end{bmatrix}^{\mathrm{T}}
$$

因此，$u^{\mathrm{T}}Ru$ 可以理解为惩罚 DFIG 无功参考和 ES 有功参考变化过快、控制动作过强。

如果只考虑状态项，优化可能倾向于使用很大的控制输入来换取快速收敛；但实际系统中，参考指令变化过大可能带来执行压力或功率冲击。因此，论文同时考虑状态代价和输入代价，以平衡：

$$
\text{快速收敛}
$$

和：

$$
\text{控制动作不过大}
$$

---

### 16.11 从式 (12) 到式 (13)

由全场闭环模型可知：

$$
u=Cx
$$

所以输入代价可以直接合并到状态二次型中：

$$
u^{\mathrm{T}}Ru
=
(Cx)^{\mathrm{T}}R(Cx)
=
x^{\mathrm{T}}C^{\mathrm{T}}RCx
$$

因此，式 (12) 可以改写为：

$$
\int_0^{\infty}
e^{2v_{\mathrm{spe}}t}
x^{\mathrm{T}}(t)
\left(
Q+C^{\mathrm{T}}RC
\right)
x(t)\,\mathrm{d}t
$$

也就是论文式 (13) 中的收敛速度目标函数：

$$
f_{\mathrm{CR}}
=

\int_0^{\infty}
e^{2v_{\mathrm{spe}}t}
x^{\mathrm{T}}(t)
\left(
Q+C^{\mathrm{T}}RC
\right)
x(t)\,\mathrm{d}t
$$

所以，式 (13) 不是新的物理假设，而是将 $u=Cx$ 代入式 (12) 后，把状态代价和输入代价合并成一个状态二次型。

可以理解为：

$$
\boxed{
\text{式 (12)：状态代价 }x^{\mathrm{T}}Qx+\text{控制输入代价 }u^{\mathrm{T}}Ru
}
$$

$$
\boxed{
\text{式 (13)：由于 }u=Cx\text{，将控制输入代价也合并为状态二次型}
}
$$

---

### 16.12 $f_{\mathrm{CR}}$ 的矩阵方程背景：从 LQR 型指标到 Lyapunov 方程

论文给出的收敛速度目标可以写成：

$$
f_{\mathrm{CR}}
=
\int_0^{\infty}
e^{2v_{\mathrm{spe}}t}
x^{\mathrm{T}}(t)
\left(
Q+C^{\mathrm{T}}RC
\right)
x(t)\,\mathrm{d}t
$$

它表面上没有显式出现 $A_{\mathrm{cl}}$，但状态轨迹 $x(t)$ 由闭环系统决定：

$$
\begin{cases}
\dot{x}=A_{\mathrm{cl}}x,\\
A_{\mathrm{cl}}=A-BC
\end{cases}
$$

因此，$A_{\mathrm{cl}}$ 是通过 $x(t)$ 间接进入 $f_{\mathrm{CR}}$ 的。通信拓扑 $M_{\mathrm{c}}$ 改变时，$C$、$A_{\mathrm{cl}}$ 和状态轨迹都会改变，最终改变该积分指标。

这里需要特别区分两种视角：

$$
\boxed{
\text{标准 LQR：直接把反馈矩阵 }C\text{ 作为连续设计变量来优化。}
}
$$

$$
\boxed{
\text{本文通信拓扑优化：把 }M_{\mathrm{c}}\text{ 作为设计变量，并通过 }C=\mathcal{H}(M_{\mathrm{c}})\text{ 间接改变 }C\text{。}
}
$$

所以，本文不是在“已知一个固定 $C$ 后再优化 $f_{\mathrm{CR}}$”；更准确地说，是在可行通信拓扑集合中改变 $M_{\mathrm{c}}$，让其诱导出的 $C$ 使 $f_{\mathrm{CR}}$ 尽可能小。

---

#### 16.12.1 普通 LQR 型二次性能指标

先不考虑指定收敛速度。对于线性系统：

$$
\dot{x}=Ax-Bu
$$

普通 LQR 型二次指标为：

$$
\int_0^\infty
\left(
x^{\mathrm{T}}Qx+u^{\mathrm{T}}Ru
\right)\,\mathrm{d}t
$$

其中，$Q\succeq 0$，$R\succ 0$。$x^{\mathrm{T}}Qx$ 惩罚状态偏差，$u^{\mathrm{T}}Ru$ 惩罚控制输入。若 $u=Cx$ 可以自由设计，标准 LQR 就是在这些权重下寻找稳定且代价最小的反馈矩阵 $C$。

> 注：$Q\succeq 0$ 表示 $Q$ 是半正定矩阵，即对任意状态向量 $x$ 都有 $x^{\mathrm{T}}Qx\geq 0$，允许某些状态方向不被惩罚；$R\succ 0$ 表示 $R$ 是正定矩阵，即对任意非零输入 $u$ 都有 $u^{\mathrm{T}}Ru>0$，表示控制输入只要不为零就会产生正代价。

---

#### 16.12.2 普通 LQR 对应的代数 Riccati 方程

对于系统：

$$
\dot{x}=Ax-Bu
$$

和性能指标：

$$
\int_0^\infty
\left(
x^{\mathrm{T}}Qx+u^{\mathrm{T}}Ru
\right)\,\mathrm{d}t
$$

对应的连续时间代数 Riccati 方程可以写成：

$$
A^{\mathrm{T}}P+PA-PBR^{-1}B^{\mathrm{T}}P+Q=0
$$

其中，$P=P^{\mathrm{T}}\succeq 0$ 是 Riccati 方程的稳定化解。在本文采用的符号约定 $\dot{x}=Ax-Bu$ 下，最优状态反馈为：

$$
\begin{cases}
u=C_{\mathrm{LQR}}x,\\
C_{\mathrm{LQR}}=R^{-1}B^{\mathrm{T}}P
\end{cases}
$$

于是闭环系统为：

$$
\dot{x}
=
\left(A-BC_{\mathrm{LQR}}\right)x
$$

这说明，在标准 LQR 中，Riccati 方程给出了一个连续自由反馈增益 $C_{\mathrm{LQR}}$，使系统在状态偏差和控制输入之间取得折中。

---

#### 16.12.3 引入指定收敛速度后的矩阵平移

论文不仅希望闭环系统稳定，还希望其收敛速度快于给定值 $v_{\mathrm{spe}}$，也就是：

$$
\operatorname{Re}
\left(
\lambda_i(A_{\mathrm{cl}})
\right)
<
-v_{\mathrm{spe}}
$$

指数权重可以通过加权状态解释。定义：

$$
\begin{cases}
z(t)=e^{v_{\mathrm{spe}}t}x(t),\\
w(t)=e^{v_{\mathrm{spe}}t}u(t)
\end{cases}
$$

若原系统为 $\dot{x}=Ax-Bu$，则：

$$
\dot{z}
=
\left(A+v_{\mathrm{spe}}I\right)z-Bw
$$

记：

$$
A_{\mathrm{v}}=A+v_{\mathrm{spe}}I
$$

则带指数权重的 LQR 型问题可以转化为移位系统：

$$
\dot{z}=A_{\mathrm{v}}z-Bw
$$

上的普通 LQR 问题。此时对应的 Riccati 方程为：

$$
A_{\mathrm{v}}^{\mathrm{T}}P
+PA_{\mathrm{v}}
-PBR^{-1}B^{\mathrm{T}}P
+Q
=0
$$

若该 Riccati 方程存在稳定化解，则：

$$
C_{\mathrm{LQR}}=R^{-1}B^{\mathrm{T}}P
$$

会使移位闭环矩阵 $A_{\mathrm{v}}-BC_{\mathrm{LQR}}$ 稳定。换回原系统，就是 $A-BC_{\mathrm{LQR}}$ 的特征值实部小于 $-v_{\mathrm{spe}}$。

因此，指数权重 $e^{2v_{\mathrm{spe}}t}$ 的作用可以理解为：通过矩阵平移 $A\rightarrow A+v_{\mathrm{spe}}I$，将“收敛速度快于 $v_{\mathrm{spe}}$”转化为移位系统稳定性问题。

---

#### 16.12.4 本文与标准 LQR 的区别

虽然论文式 (12)–(13) 具有 LQR 型性能指标结构，但本文并不是直接求解标准 LQR 反馈矩阵。

标准 LQR 中，反馈矩阵可以自由取为：

$$
C_{\mathrm{LQR}}
=
R^{-1}B^{\mathrm{T}}P
$$

但本文中的反馈矩阵必须由通信拓扑决定：

$$
C=\mathcal{H}(M_{\mathrm{c}})
$$

而通信邻接矩阵 $M_{\mathrm{c}}$ 又受到以下限制：

1. 邻接矩阵元素为 $0$ 或 $1$；
2. 通信图必须连通；
3. 通信链路数量应尽可能少；
4. 节点度分布需要兼顾通信生存性；
5. $C$ 的结构必须符合一致性控制律和 leader-follower 结构。

因此，本文不能简单地令 $C=C_{\mathrm{LQR}}$，而是要在可行通信拓扑集合中寻找合适的 $M_{\mathrm{c}}$。每一个候选拓扑都会诱导出一个反馈矩阵：

$$
C=\mathcal{H}(M_{\mathrm{c}})
$$

再由这个 $C$ 形成闭环矩阵：

$$
A_{\mathrm{cl}}=A-BC
$$

然后用 $f_{\mathrm{CR}}$ 评价其收敛性能。也就是说，优化变量不是连续的 $C$，而是受通信约束的 $M_{\mathrm{c}}$；$C$ 是由 $M_{\mathrm{c}}$ 间接得到的结果。

---

#### 16.12.5 给定通信拓扑时与 Lyapunov 方程的关系

如果通信拓扑 $M_{\mathrm{c}}$ 已经给定，则反馈矩阵 $C$ 也就确定了。此时 $f_{\mathrm{CR}}$ 不再是用来“设计 $C$”的目标，而是用来“评价该拓扑对应闭环性能”的指标。此时：

$$
A_{\mathrm{cl}}=A-BC
$$

令：

$$
A_{\mathrm{cl},v}
=
A_{\mathrm{cl}}+v_{\mathrm{spe}}I
$$

并令：

$$
Q_C
=
Q+C^{\mathrm{T}}RC
$$

用 $z(t)=e^{v_{\mathrm{spe}}t}x(t)$ 表示加权状态，则：

$$
\begin{cases}
f_{\mathrm{CR}}
=
\displaystyle
\int_0^{\infty}
z^{\mathrm{T}}(t)Q_Cz(t)\,\mathrm{d}t,\\
\dot{z}=A_{\mathrm{cl},v}z
\end{cases}
$$

若 $A_{\mathrm{cl},v}$ 是 Hurwitz 矩阵，则该积分可以通过连续时间 Lyapunov 方程评价。令 $P=P^{\mathrm{T}}\succeq 0$，则：

$$
A_{\mathrm{cl},v}^{\mathrm{T}}P
+PA_{\mathrm{cl},v}
+Q_C
=0
$$

也就是：

$$
\left(A_{\mathrm{cl}}+v_{\mathrm{spe}}I\right)^{\mathrm{T}}P
+P\left(A_{\mathrm{cl}}+v_{\mathrm{spe}}I\right)
+Q+C^{\mathrm{T}}RC
=0
$$

对于给定初始状态 $x_0$，性能指标可写成：

$$
f_{\mathrm{CR}}=x_0^{\mathrm{T}}Px_0
$$

如果考虑一组初始状态，其协方差为 $\Gamma$，则常可写成：

$$
\operatorname{tr}(P\Gamma)
$$

这里的 $\Gamma$ 可以理解为初始状态 $x_0$ 的协方差矩阵，用来描述“一组可能初始扰动”的分布范围和方向。通常这里默认 $x_0$ 是零均值的偏差变量；如果初始偏差均值不为零，还需要额外考虑均值项。如果单个初始状态的代价是：

$$
f_{\mathrm{CR}}=x_0^{\mathrm{T}}Px_0
$$

那么对一组随机初始状态取平均时，有：

$$
\mathbb{E}
\left[
x_0^{\mathrm{T}}Px_0
\right]
=
\operatorname{tr}
\left(
P\Gamma
\right)
$$

其意义是：不再只评价某一个特定初始扰动 $x_0$ 下的闭环性能，而是评价一组可能扰动下的平均闭环性能。如果取 $\Gamma=I$，则该指标退化为 $\operatorname{tr}(P)$，可以理解为各个状态方向上闭环代价的总和；如果某些状态方向更容易发生扰动，或更值得关注，就可以在 $\Gamma$ 中给这些方向更大的权重。

因此，从“给定拓扑后评价闭环性能”的角度看，$f_{\mathrm{CR}}$ 更直接对应 Lyapunov 方程；只有在“反馈增益完全自由、需要求最优反馈”时，才对应 Riccati 方程。

放回本文的通信网络优化问题中，实际流程可以理解为：

$$
\boxed{
\text{给定候选 }M_{\mathrm{c}}
\rightarrow
C=\mathcal{H}(M_{\mathrm{c}})
\rightarrow
A_{\mathrm{cl}}=A-BC
\rightarrow
\text{用 Lyapunov 方程计算 }f_{\mathrm{CR}}
}
$$

然后在不同候选通信拓扑之间比较 $f_{\mathrm{CR}}$，选出收敛性能更好的拓扑。

---

#### 16.12.6 一个极简数值例子：$M_{\mathrm{c}}$ 如何影响 $f_{\mathrm{CR}}$

为了更直观看到通信拓扑如何影响最终指标，可以构造一个非常简化的三节点一致性模型。这里不复现论文中的 DFIG/ES 完整状态空间模型，而只保留以下逻辑链条：

$$
M_{\mathrm{c}}
\rightarrow
L_{\mathrm{c}}
\rightarrow
C
\rightarrow
A_{\mathrm{cl}}
\rightarrow
f_{\mathrm{CR}}
$$

示例中令：

$$
A=0,\quad B=I,\quad C=L_{\mathrm{c}}+G_{\mathrm{pin}}
$$

其中，$G_{\mathrm{pin}}=\operatorname{diag}(1,0,0)$ 表示第 1 个节点被 leader pinning。这里把 pinning 矩阵记为 $G_{\mathrm{pin}}$，是为了避免和前面表示初始状态协方差的 $\Gamma$ 混淆。

对应的 Python 代码可以写成：

```python
import numpy as np
from scipy.linalg import solve_continuous_lyapunov, eigvals


def calc_f_CR(Mc, v_spe=0.18, r=0.02):
    n = Mc.shape[0]

    Lambda = np.diag(Mc.sum(axis=1))
    Lc = Lambda - Mc

    G_pin = np.diag([1, 0, 0])

    A = np.zeros((n, n))
    B = np.eye(n)
    C = Lc + G_pin

    Acl = A - B @ C
    Abar = Acl + v_spe * np.eye(n)

    Q = np.eye(n)
    R = r * np.eye(n)
    Qcl = Q + C.T @ R @ C

    eig_Acl = eigvals(Acl)

    if np.max(np.real(eigvals(Abar))) >= 0:
        return Lc, C, eig_Acl, np.inf

    P = solve_continuous_lyapunov(Abar.T, -Qcl)

    x0 = np.array([1.0, -1.0, 0.5])
    f_CR = float(x0.T @ P @ x0)

    return Lc, C, eig_Acl, f_CR
```

用三种通信拓扑测试：

```python
Mc_chain = np.array([
    [0, 1, 0],
    [1, 0, 1],
    [0, 1, 0]
], dtype=float)

Mc_complete = np.array([
    [0, 1, 1],
    [1, 0, 1],
    [1, 1, 0]
], dtype=float)

Mc_disconnected = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 0]
], dtype=float)
```

在本机 `D:\CondaData\envs\rl-grid\python.exe` 环境中计算得到：

| 拓扑 | $\operatorname{Re}(\lambda(A_{\mathrm{cl}}))$ | $f_{\mathrm{CR}}$ |
|---|---:|---:|
| chain | $[-3.246980,\ -1.554958,\ -0.198062]$ | $0.752468$ |
| complete | $[-3.732051,\ -3.000000,\ -0.267949]$ | $0.554915$ |
| disconnected | $[-2.618034,\ -0.381966,\ 0.000000]$ | $\infty$ |

这个例子说明：

1. 不同 $M_{\mathrm{c}}$ 会产生不同的 $L_{\mathrm{c}}$；
2. 不同 $L_{\mathrm{c}}$ 会改变 $C=L_{\mathrm{c}}+G_{\mathrm{pin}}$；
3. 不同 $C$ 会改变闭环矩阵 $A_{\mathrm{cl}}=A-BC$；
4. 如果 $A_{\mathrm{cl}}+v_{\mathrm{spe}}I$ 不是 Hurwitz 矩阵，则带指数权重的积分发散，$f_{\mathrm{CR}}=\infty$；
5. 在这个极简模型中，complete 拓扑比 chain 拓扑得到更小的 $f_{\mathrm{CR}}$，说明其闭环收敛性能指标更好。

需要注意，这个例子只是为了说明机制，不是论文完整模型的复现。真实论文中，$A$、$B$ 和 $C$ 来自 DFIG/ES 的状态空间模型、一致性控制律、leader-follower 结构以及通信拓扑约束；这里的 $A=0$、$B=I$、$C=L_{\mathrm{c}}+G_{\mathrm{pin}}$ 只是为了把“拓扑改变 $\rightarrow$ 指标改变”的路径单独展示出来。

---

### 16.13 当前理解小结

论文中的收敛速度目标可以概括为：

$$
\boxed{
\text{它是带指定指数收敛速度要求的 LQR 型闭环性能指标。}
}
$$

式 (12) 到式 (13) 的关键只是将：

$$
u=Cx
$$

代入原来的状态项与输入项二次指标，从而得到：

$$
f_{\mathrm{CR}}
=
\int_0^{\infty}
e^{2v_{\mathrm{spe}}t}
x^{\mathrm{T}}(t)
\left(
Q+C^{\mathrm{T}}RC
\right)
x(t)\,\mathrm{d}t
$$

其中，指数权重：

$$
e^{2v_{\mathrm{spe}}t}
$$

等价于引入加权状态 $z(t)=e^{v_{\mathrm{spe}}t}x(t)$，从而把“收敛速度快于 $v_{\mathrm{spe}}$”转化为移位闭环矩阵：

$$
A_{\mathrm{cl}}+v_{\mathrm{spe}}I
$$

的稳定性要求。

Riccati 方程和 Lyapunov 方程的关系可以理解为：

$$
\boxed{
\text{反馈增益 }C\text{ 自由时，对应 Riccati 方程直接求最优 }C_{\mathrm{LQR}}\text{。}
}
$$

$$
\boxed{
\text{拓扑 }M_{\mathrm{c}}\text{ 给定时，}C=\mathcal{H}(M_{\mathrm{c}})\text{ 已确定，对应 Lyapunov 方程评价 }f_{\mathrm{CR}}\text{。}
}
$$

若反馈增益完全自由，则可对移位系统使用标准 LQR，对应 Riccati 方程：

$$
\begin{cases}
A_{\mathrm{v}}^{\mathrm{T}}P
+PA_{\mathrm{v}}
-PBR^{-1}B^{\mathrm{T}}P
+Q
=0,\\
A_{\mathrm{v}}=A+v_{\mathrm{spe}}I
\end{cases}
$$

但在本文中，反馈矩阵不是自由变量，而是由通信拓扑决定：

$$
C=\mathcal{H}(M_{\mathrm{c}})
$$

因此，对于给定通信拓扑，更自然的是用 Lyapunov 方程评价该拓扑诱导出的闭环指标：

$$
\left(A_{\mathrm{cl}}+v_{\mathrm{spe}}I\right)^{\mathrm{T}}P
+P\left(A_{\mathrm{cl}}+v_{\mathrm{spe}}I\right)
+Q+C^{\mathrm{T}}RC
=0
$$

这部分的整体逻辑可以概括为：

$$
M_{\mathrm{c}}
\rightarrow
C
\rightarrow
A_{\mathrm{cl}}
\rightarrow
x(t)
\rightarrow
f_{\mathrm{CR}}
$$

所以，“优化 $f_{\mathrm{CR}}$” 在本文中不是直接调节一个已经自由的 $C$，而是在可行通信拓扑集合中寻找更合适的 $M_{\mathrm{c}}$。每个 $M_{\mathrm{c}}$ 对应一个 $C$，每个 $C$ 对应一个闭环矩阵 $A_{\mathrm{cl}}$，最终通过 $f_{\mathrm{CR}}$ 比较闭环收敛性能。

虽然 $f_{\mathrm{CR}}$ 的表达式中没有显式出现 $A_{\mathrm{cl}}$，但 $A_{\mathrm{cl}}$ 通过状态轨迹 $x(t)$ 进入积分性能指标。因此，优化 $f_{\mathrm{CR}}$ 本质上是在优化通信拓扑诱导的闭环动态性能。

## 17. 通信稀疏性目标的理解

Section III-B 中，论文提出的第二个优化目标是通信稀疏性。其出发点是：风电场中 WT/ES 节点数量较多，如果通信链路过多，会增加通信负担、通信能量损耗和网络维护成本。因此，在保证控制性能和通信连通性的前提下，希望通信网络尽可能稀疏。

论文将通信稀疏性目标写为：

$$
f_{\mathrm{CS}}
=\|\Lambda_{\mathrm{c}}\|_{\ell_1}
=\sum_{i=1}^{N}
|\Lambda_{\mathrm{c},ii}|
=
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}
$$

其中，$\Lambda_{\mathrm{c}}$ 是通信网络的度矩阵，$\Lambda_{\mathrm{c},ii}$ 是第 $i$ 个节点的度，即该节点直接连接的通信邻居数量。

更严谨地说，可以把度矩阵的对角元素抽成度向量：

$$
d_{\mathrm{c}}
=
\begin{bmatrix}
\Lambda_{\mathrm{c},11}&
\Lambda_{\mathrm{c},22}&
\cdots&
\Lambda_{\mathrm{c},NN}
\end{bmatrix}^{\mathrm{T}}
$$

则论文中的 $\|\Lambda_{\mathrm{c}}\|_{\ell_1}$ 可以理解为：

$$
\|\Lambda_{\mathrm{c}}\|_{\ell_1}
\equiv
\|d_{\mathrm{c}}\|_{\ell_1}
$$

也就是对所有节点度数求 $\ell_1$ 范数。

由于节点度数非负：

$$
\Lambda_{\mathrm{c},ii}\geq 0
$$

所以有：

$$
|\Lambda_{\mathrm{c},ii}|
=

\Lambda_{\mathrm{c},ii}
$$

因此，论文中的 $f_{\mathrm{CS}}$ 本质上就是所有节点度数之和。

---

### 17.1 为什么使用 $\ell_1$ 范数

在优化问题中，$\ell_1$ 范数常被用来促进稀疏性。对于一个向量：

$$
z=
\begin{bmatrix}
z_1&
z_2&
\cdots&
z_N
\end{bmatrix}^{\mathrm{T}}
$$

其 $\ell_1$ 范数为：

$$
\|z\|_{\ell_1}
=
\sum_{i=1}^{N}|z_i|
$$

如果 $z_i$ 是非负变量，则：

$$
\|z\|_{\ell_1}
=
\sum_{i=1}^{N}z_i
$$

在本文中，度矩阵 $\Lambda_{\mathrm{c}}$ 的对角元素就是每个通信节点的度数，因此：

$$
\|\Lambda_{\mathrm{c}}\|_{\ell_1}
=
\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
$$

可以理解为对所有节点通信邻居数量求和。

这里需要注意，真正直接表示“非零元素个数”的是 $\ell_0$ “范数”。例如，对向量 $z$ 有：

$$
\|z\|_{\ell_0}
=
\#\{i:z_i\neq 0\}
$$

> 注：这里的 $\#\{\cdot\}$ 表示集合中元素的个数，也就是集合的基数。例如 $\#\{i:z_i\neq 0\}$ 表示满足 $z_i\neq 0$ 的下标数量。类似含义也常写成 $\operatorname{card}\{i:z_i\neq 0\}$；而 `\card` 通常不是 LaTeX 默认命令，需要作者自己定义宏。

如果直接把通信邻接矩阵中的非零元素计数，也确实可以得到通信边数信息。但是 $\ell_0$ 形式通常是非凸、非光滑的组合优化目标，不利于后续用连续优化或 ADMM 类方法处理。因此，稀疏优化中常用 $\ell_1$ 范数作为替代指标。

在本文这个问题中，$\ell_1$ 还有一个更直接的含义：通信边本身由 $0$-$1$ 邻接矩阵描述，节点度数 $\Lambda_{\mathrm{c},ii}$ 又是非负整数，所以对度数取 $\ell_1$ 范数并不是单纯的“近似稀疏性”，而是直接等于所有节点度数之和。

也就是说：

$$
\boxed{
\|\Lambda_{\mathrm{c}}\|_{\ell_1}
=
\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
\text{ 直接度量全网通信连接负担}
}
$$

如果对度向量使用 $\ell_0$，反而会得到“有至少一个邻居的节点数”，这并不能表示通信边数。例如一个节点有 $1$ 个邻居和有 $10$ 个邻居，在度向量的 $\ell_0$ 计数中都只贡献 $1$，无法反映通信负担差异。因此，对通信链路稀疏性来说，度数的 $\ell_1$ 和边数之间的关系更合适。

还要区分“向量 $\ell_1$ 范数”和“矩阵诱导 1-范数”。矩阵诱导 1-范数定义为：

$$
\|A\|_1
=
\max_{x\neq 0}
\frac{\|Ax\|_1}{\|x\|_1}
=
\max_j
\sum_i |a_{ij}|
$$

也就是矩阵各列绝对值列和的最大值。如果把这个定义用于对角度矩阵 $\Lambda_{\mathrm{c}}$，则有：

$$
\|\Lambda_{\mathrm{c}}\|_1
=
\max_i \Lambda_{\mathrm{c},ii}
$$

它表示最大节点度，而不是所有节点度数之和。因此，论文这里的 $\|\Lambda_{\mathrm{c}}\|_{\ell_1}$ 更应理解为“把度矩阵的对角元素看成度向量后取 $\ell_1$ 范数”，而不是严格意义上的矩阵诱导 1-范数。

---

### 17.2 通信稀疏性的物理含义

节点度数 $\Lambda_{\mathrm{c},ii}$ 表示第 $i$ 个节点需要直接通信的邻居数量。若某个节点度数越大，说明它需要维护的通信连接越多。

因此：

$$
\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
$$

越大，表示整个通信网络中总的通信连接负担越重；反之，该值越小，表示网络越稀疏。

所以，最小化：

$$
f_{\mathrm{CS}}
=
\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
$$

就等价于减少通信网络中的总连接数量，从而降低通信负担和通信能量损耗。

---

### 17.3 图论基本定理：无向图中所有节点度数之和等于边数的两倍

通信稀疏性目标之所以能够表示通信链路数量，关键依赖于图论中的一个基本定理：

$$
\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
=
2|E_{\mathrm{c}}|
$$

其中，$|E_{\mathrm{c}}|$ 表示通信图中的边数。

证明如下。

设通信网络为无向图：

$$
G_{\mathrm{c}}=(V_{\mathrm{c}},E_{\mathrm{c}})
$$

其中，$V_{\mathrm{c}}$ 是节点集合，$E_{\mathrm{c}}$ 是边集合。

对于任意一条无向边：

$$
e=(v_i,v_j)
$$

它连接节点 $v_i$ 和节点 $v_j$。因此，这条边会使节点 $v_i$ 的度增加 $1$，同时也会使节点 $v_j$ 的度增加 $1$。

也就是说，每一条无向边对全图所有节点度数之和的贡献为：

$$
1+1=2
$$

如果图中一共有 $|E_{\mathrm{c}}|$ 条边，则所有边对节点度数之和的总贡献为：

$$
2|E_{\mathrm{c}}|
$$

因此得到：

$$
\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
=
2|E_{\mathrm{c}}|
$$

这一定理也可以从邻接矩阵角度理解。对于无向无权图，邻接矩阵满足：

$$
M_{\mathrm{c},ij}
=
M_{\mathrm{c},ji}
$$

且若节点 $i$ 与节点 $j$ 相连，则：

$$
M_{\mathrm{c},ij}=M_{\mathrm{c},ji}=1
$$

因此，每一条无向边会在邻接矩阵中产生两个 $1$，分别位于 $(i,j)$ 和 $(j,i)$ 位置。

节点 $i$ 的度为：

$$
\Lambda_{\mathrm{c},ii}
=
\sum_{j=1}^{N}M_{\mathrm{c},ij}
$$

所以所有节点度数之和为：

$$
\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
=
\sum_{i=1}^{N}
\sum_{j=1}^{N}
M_{\mathrm{c},ij}
$$

由于每条边在邻接矩阵中被计数两次，因此：

$$
\sum_{i=1}^{N}
\sum_{j=1}^{N}
M_{\mathrm{c},ij}
=
2|E_{\mathrm{c}}|
$$

故有：

$$
\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
=
2|E_{\mathrm{c}}|
$$

因此，对于无向无权图，也可以直接用邻接矩阵元素之和表示通信边数：

$$
\sum_{i=1}^{N}
\sum_{j=1}^{N}
M_{\mathrm{c},ij}
=
2|E_{\mathrm{c}}|
$$

这与对度矩阵对角元素求和是等价的。论文使用 $\|\Lambda_{\mathrm{c}}\|_{\ell_1}$，本质上与对邻接矩阵求和等价，但从表达上更突出“节点通信负担”的含义：每个节点的度数就是该节点需要维护的直接通信邻居数量。

于是论文中的通信稀疏性目标可以写成：

$$
f_{\mathrm{CS}}
=
\|\Lambda_{\mathrm{c}}\|_{\ell_1}
=2|E_{\mathrm{c}}|
$$

因此，最小化 $f_{\mathrm{CS}}$ 等价于最小化通信边数：

$$
\min f_{\mathrm{CS}}
\Longleftrightarrow
\min |E_{\mathrm{c}}|
$$

这就是该目标能够体现通信稀疏性的根本原因。

---

### 17.4 当前理解小结

通信稀疏性目标为：

$$
f_{\mathrm{CS}}
=\|\Lambda_{\mathrm{c}}\|_{\ell_1}
=\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
$$

由于无向图中所有节点度数之和等于边数的两倍：

$$
\sum_{i=1}^{N}\Lambda_{\mathrm{c},ii}
=
2|E_{\mathrm{c}}|
$$

所以：

$$
f_{\mathrm{CS}}
=
2|E_{\mathrm{c}}|
$$

因此，最小化 $f_{\mathrm{CS}}$ 实际上就是减少通信网络中的边数，从而降低通信负担和通信能量损耗。

需要注意，$\|\Lambda_{\mathrm{c}}\|_{\ell_1}$ 在这里应理解为度向量 $d_{\mathrm{c}}$ 的 $\ell_1$ 范数，而不是矩阵诱导 1-范数。若按矩阵诱导 1-范数理解，$\|\Lambda_{\mathrm{c}}\|_1$ 表示最大节点度，而不是通信边数。

这一目标与前面的收敛速度目标存在天然矛盾：通信边越多，通常信息传播越快、控制收敛越好；但通信边越多，通信成本和负担也越高。因此，论文需要在收敛性能和通信稀疏性之间进行折中优化。

## 18. 通信生存性目标的理解

Section III-B 中，论文提出的第三个优化目标是通信生存性。其基本出发点是：在大规模风电场中，WT/ES 节点数量较多，局部通信节点故障的概率不可忽视。如果通信网络过度依赖少数高节点度节点，那么这些节点一旦故障，就可能同时影响大量通信链路，从而削弱分布式一致性控制的有效性。

因此，论文希望通信网络不仅要连通、收敛快、边数少，还要避免形成过强的中心节点，使通信负担在各节点之间尽可能均匀分布。

论文将通信生存性目标写为：

$$
f_{\mathrm{NF}}
=
\|\Lambda_{\mathrm{c}}-\bar{\Lambda}_{\mathrm{c}}I\|_{\ell_1}
=\sum_{i=1}^{N}
\left|
\Lambda_{\mathrm{c},ii}
-
\bar{\Lambda}_{\mathrm{c}}
\right|
$$

其中，$\Lambda_{\mathrm{c}}$ 是通信网络的度矩阵，$\Lambda_{\mathrm{c},ii}$ 表示第 $i$ 个节点的度，$\bar{\Lambda}_{\mathrm{c}}$ 表示所有节点度数的平均值：

$$
\bar{\Lambda}_{\mathrm{c}}
=
\frac{1}{N}
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}
$$

更严谨地说，这里的 $\|\Lambda_{\mathrm{c}}-\bar{\Lambda}_{\mathrm{c}}I\|_{\ell_1}$ 应理解为对角元素组成的度偏差向量的 $\ell_1$ 范数，而不是矩阵诱导 1-范数。若按矩阵诱导 1-范数理解，得到的是最大绝对度偏差，而不是所有节点度偏差的总量。

令：

$$
d_{\Delta}
=
\begin{bmatrix}
\Lambda_{\mathrm{c},11}-\bar{\Lambda}_{\mathrm{c}}&
\Lambda_{\mathrm{c},22}-\bar{\Lambda}_{\mathrm{c}}&
\cdots&
\Lambda_{\mathrm{c},NN}-\bar{\Lambda}_{\mathrm{c}}
\end{bmatrix}^{\mathrm{T}}
$$

则：

$$
f_{\mathrm{NF}}
=
\|d_{\Delta}\|_{\ell_1}
$$

---

### 18.1 通信生存性为什么与节点度有关

节点度 $\Lambda_{\mathrm{c},ii}$ 表示第 $i$ 个节点直接连接的通信邻居数量。节点度越大，说明该节点需要维护和处理的通信连接越多，其通信负担也越重。

如果某些节点度数显著高于其他节点，则通信网络呈现较强的中心化特征。这类高节点度节点承担更多信息交换任务，通信负担更重；一旦发生通信故障，也会同时影响更多通信链路，对网络连通性和一致性控制造成更大冲击。

因此，通信生存性目标的核心并不是单纯增加通信边数，而是希望在已有通信边数条件下，使边的分布更加均衡，避免网络过度依赖少数中心节点。

可以将其理解为一种“去中心化”的设计目标：

$$
\boxed{
\text{避免通信连接过度集中在少数节点上，使通信负担在各节点间更均匀分布。}
}
$$

---

### 18.2 为什么构造 $\Lambda_{\mathrm{c}}-\bar{\Lambda}_{\mathrm{c}}I$

度矩阵可以写成：

$$
\Lambda_{\mathrm{c}}
=
\operatorname{diag}
\left(
\Lambda_{\mathrm{c},11},
\Lambda_{\mathrm{c},22},
\cdots,
\Lambda_{\mathrm{c},NN}
\right)
$$

平均度对应的矩阵为：

$$
\bar{\Lambda}_{\mathrm{c}}I
=
\operatorname{diag}
\left(
\bar{\Lambda}_{\mathrm{c}},
\bar{\Lambda}_{\mathrm{c}},
\cdots,
\bar{\Lambda}_{\mathrm{c}}
\right)
$$

因此：

$$
\Lambda_{\mathrm{c}}-\bar{\Lambda}_{\mathrm{c}}I
=\operatorname{diag}
\left(
\Lambda_{\mathrm{c},11}-\bar{\Lambda}_{\mathrm{c}},
\Lambda_{\mathrm{c},22}-\bar{\Lambda}_{\mathrm{c}},
\cdots,
\Lambda_{\mathrm{c},NN}-\bar{\Lambda}_{\mathrm{c}}
\right)
$$

该矩阵的对角元素表示每个节点度数相对于平均度的偏差。若 $\Lambda_{\mathrm{c},ii}>\bar{\Lambda}_{\mathrm{c}}$，说明第 $i$ 个节点的通信负担高于平均水平；若 $\Lambda_{\mathrm{c},ii}<\bar{\Lambda}_{\mathrm{c}}$，说明该节点的通信连接偏少。

如果所有节点度数完全相等，则有：

$$
\Lambda_{\mathrm{c},ii}
=
\bar{\Lambda}_{\mathrm{c}},
\quad
\forall i
$$

此时：

$$
\Lambda_{\mathrm{c}}-\bar{\Lambda}_{\mathrm{c}}I=0
$$

并且：

$$
f_{\mathrm{NF}}=0
$$

所以，该目标函数衡量的是节点度分布偏离均匀程度的总量，而不是通信边数本身。

---

### 18.3 为什么使用 $\ell_1$ 范数

由于 $\Lambda_{\mathrm{c}}-\bar{\Lambda}_{\mathrm{c}}I$ 是对角矩阵，其主要信息就在对角线上。论文中的 $\ell_1$ 范数实质上是对所有节点度偏差取绝对值后求和：

$$
f_{\mathrm{NF}}
=
\sum_{i=1}^{N}
\left|
\Lambda_{\mathrm{c},ii}
-
\bar{\Lambda}_{\mathrm{c}}
\right|
$$

这可以理解为节点度相对平均度的总绝对偏差。

这里必须使用绝对值或类似的非负度量。若直接对偏差求和，则由平均度定义可得：

$$
\sum_{i=1}^{N}
\left(
\Lambda_{\mathrm{c},ii}
-
\bar{\Lambda}_{\mathrm{c}}
\right)
=
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}
-
N\bar{\Lambda}_{\mathrm{c}}
=
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}
-
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}
=0
$$

也就是说，若不取绝对值，正偏差和负偏差会相互抵消，结果恒为 $0$，无法反映节点度分布是否均衡。

采用 $\ell_1$ 范数后，每个节点的偏差都以非负形式计入目标函数：

$$
\left|
\Lambda_{\mathrm{c},ii}
-
\bar{\Lambda}_{\mathrm{c}}
\right|
\geq 0
$$

因此，$f_{\mathrm{NF}}$ 越小，说明各节点度数越接近平均度；$f_{\mathrm{NF}}$ 越大，说明通信连接越集中在部分节点上，网络中心化程度越强。

从优化角度看，$\ell_1$ 范数还有一个优点：它是分段线性的，通常比平方偏差形式更容易与混合整数优化或 ADMM 分解框架结合。

---

### 18.4 该指标与通信稀疏性的区别

通信稀疏性目标为：

$$
f_{\mathrm{CS}}
=
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}
=
2|E_{\mathrm{c}}|
$$

它关注的是通信边的总数量，即“边有多少”。只要边数相同，$f_{\mathrm{CS}}$ 就相同。

而通信生存性目标为：

$$
f_{\mathrm{NF}}
=
\sum_{i=1}^{N}
\left|
\Lambda_{\mathrm{c},ii}
-
\bar{\Lambda}_{\mathrm{c}}
\right|
$$

它关注的是通信边在不同节点之间的分布，即“边分布在哪里”。

因此，两个目标关注点不同：

$$
\boxed{
f_{\mathrm{CS}}\text{ 关注“边有多少”}
}
$$

$$
\boxed{
f_{\mathrm{NF}}\text{ 关注“边分布得是否均匀”}
}
$$

即使两个通信网络具有相同或相近的边数，如果一个网络的边高度集中在少数节点上，而另一个网络的边在各节点之间分布更均衡，那么二者的通信稀疏性可能相近，但通信生存性目标会明显不同。

论文引入 $f_{\mathrm{NF}}$，正是为了补充 $f_{\mathrm{CS}}$ 无法反映的节点度不均衡问题。

---

### 18.5 该指标的局限性

需要注意，$f_{\mathrm{NF}}$ 是通信生存性的一个简化代理指标，而不是严格的节点故障鲁棒性判据。

严格的通信生存性可以考虑：

1. 删除任意一个节点后，网络是否仍然连通；
2. 删除节点后代数连通度 $\lambda_2(L_{\mathrm{c}})$ 下降多少；
3. 网络的节点连通度或边连通度；
4. 是否存在割点、关键节点或多节点故障场景。

这些指标能够更直接地描述节点故障后的网络保持能力，但通常会使优化问题更加复杂。

论文采用节点度均衡目标，是一种更容易建模和优化的处理方式。其工程含义是：

$$
\boxed{
\text{通过抑制高节点度中心节点，降低局部节点故障对通信网络和一致性控制的影响。}
}
$$

因此，$f_{\mathrm{NF}}$ 可以理解为提高通信生存性的近似指标，而不是对任意节点故障后网络仍连通的严格保证。

---

### 18.6 当前理解小结

通信生存性目标为：

$$
f_{\mathrm{NF}}
=
\|\Lambda_{\mathrm{c}}-\bar{\Lambda}_{\mathrm{c}}I\|_{\ell_1}
=
\sum_{i=1}^{N}
\left|
\Lambda_{\mathrm{c},ii}
-
\bar{\Lambda}_{\mathrm{c}}
\right|
$$

它衡量的是各节点度数相对于平均度的总绝对偏差。

因此，该目标可以理解为一种“去中心化”的通信网络设计指标：

$$
\boxed{
f_{\mathrm{NF}}\text{ 越小，节点度分布越均衡，网络对少数中心节点的依赖越弱。}
}
$$

它与通信稀疏性目标共同作用：

$$
f_{\mathrm{CS}}
\Rightarrow
\text{减少通信边数}
$$

$$
f_{\mathrm{NF}}
\Rightarrow
\text{均衡通信负担}
$$

二者结合后，论文希望得到一种既不过度密集、又不过度中心化的通信网络拓扑，从而在降低通信负担的同时提高局部节点故障下的通信生存性。

## 19. 通信拓扑约束与连通性条件的等价变换

Section III-B 中，在给出三个优化目标之后，论文进一步给出了通信拓扑优化问题中的约束条件。这些约束主要用于保证通信邻接矩阵 $M_{\mathrm{c}}$、度矩阵 $\Lambda_{\mathrm{c}}$ 和拉普拉斯矩阵 $L_{\mathrm{c}}$ 的图论含义成立，并保证通信网络连通。

---

### 19.1 通信邻接矩阵的基本约束

通信网络被建模为无向无权图，因此其邻接矩阵 $M_{\mathrm{c}}$ 需要满足：

$$
M_{\mathrm{c}}=M_{\mathrm{c}}^{\mathrm{T}}
$$

该约束表示通信关系是双向的，即若节点 $i$ 能与节点 $j$ 通信，则节点 $j$ 也能与节点 $i$ 通信。

同时，节点不能与自身建立通信边，因此：

$$
M_{\mathrm{c},ii}=0,
\quad
i=1,2,\cdots,N
$$

此外，通信边只有“存在”和“不存在”两种情况，因此：

$$
M_{\mathrm{c},ij}\in\{0,1\}
$$

其中，$M_{\mathrm{c},ij}=1$ 表示节点 $i$ 和节点 $j$ 之间存在通信链路，$M_{\mathrm{c},ij}=0$ 表示二者之间不存在直接通信链路。

因此，$M_{\mathrm{c}}$ 是一个对称的 $0$-$1$ 矩阵，且主对角元素为零。

---

### 19.2 度矩阵和拉普拉斯矩阵约束

通信度矩阵 $\Lambda_{\mathrm{c}}$ 是对角矩阵，其第 $i$ 个对角元素表示节点 $i$ 的度，即节点 $i$ 直接连接的通信邻居数量：

$$
\Lambda_{\mathrm{c},ii}
=
\sum_{j=1}^{N}
M_{\mathrm{c},ij}
$$

非对角元素为零：

$$
\Lambda_{\mathrm{c},ij}=0,
\quad
i\neq j
$$

通信拉普拉斯矩阵定义为：

$$
L_{\mathrm{c}}
=
\Lambda_{\mathrm{c}}-M_{\mathrm{c}}
$$

因此，$L_{\mathrm{c}}$ 的对角元素为节点度数，非对角元素在存在通信边时为 $-1$，不存在通信边时为 $0$。

这些约束只是把“通信图”翻译成矩阵形式，本身比较直观。真正比较关键的是后面对通信图连通性的处理。

---

### 19.3 连通性条件：从 $\lambda_2(L_{\mathrm{c}})>0$ 到矩阵不等式

对于无向图的拉普拉斯矩阵 $L_{\mathrm{c}}$，其特征值可以按从小到大排列为：

$$
0=\lambda_1(L_{\mathrm{c}})
\leq
\lambda_2(L_{\mathrm{c}})
\leq
\cdots
\leq
\lambda_N(L_{\mathrm{c}})
$$

其中，$\lambda_2(L_{\mathrm{c}})$ 是代数连通度，也称 Fiedler 值。图连通当且仅当：

$$
\lambda_2(L_{\mathrm{c}})>0
$$

如果直接在优化问题中写 $\lambda_2(L_{\mathrm{c}})>0$，这是一个特征值约束，处理起来不够方便。论文参考 [29，S. Boyd, 2006, “Convex optimization of graph Laplacian eigenvalues”] 中的思想，将其转化为半正定约束。为了先理解这个变换的核心，可以从一个固定投影形式看起。

关键观察是，拉普拉斯矩阵总满足：

$$
L_{\mathrm{c}}1_N=0
$$

其中：

$$
1_N=
\begin{bmatrix}
1&1&\cdots&1
\end{bmatrix}^{\mathrm{T}}
$$

也就是说，全 1 向量 $1_N$ 对应拉普拉斯矩阵的零特征值。这一零特征值是拉普拉斯矩阵固有的；无论图是否连通，都有 $L_{\mathrm{c}}1_N=0$，所以不能直接要求：

$$
L_{\mathrm{c}}\succ0
$$

因此，$L_{\mathrm{c}}$ 至多是半正定矩阵。

为去掉这个固有零特征值的影响，可以引入矩阵：

$$
J=
\frac{1}{N}1_N1_N^{\mathrm{T}}
$$

该矩阵是到全 1 向量方向上的正交投影矩阵。它具有如下性质：

$$
J1_N=1_N
$$

而对于任意满足：

$$
1_N^{\mathrm{T}}y=0
$$

的向量 $y$，有：

$$
Jy=0
$$

也就是说，$J$ 只作用在 $1_N$ 方向上，而在与 $1_N$ 正交的子空间上为零。

因此，考虑矩阵：

$$
L_{\mathrm{c}}+J
=
L_{\mathrm{c}}
+
\frac{1}{N}1_N1_N^{\mathrm{T}}
$$

在 $1_N$ 方向上，有：

$$
\left(
L_{\mathrm{c}}+J
\right)1_N
=
L_{\mathrm{c}}1_N
+
J1_N
=0+1_N=1_N
$$

所以 $1_N$ 方向对应的特征值由原来的 $0$ 被提升为 $1$。

而在与 $1_N$ 正交的子空间上，若 $1_N^{\mathrm{T}}y=0$，则：

$$
Jy=0
$$

因此：

$$
\left(
L_{\mathrm{c}}+J
\right)y
=
L_{\mathrm{c}}y
$$

也就是说，在该子空间上，$L_{\mathrm{c}}+J$ 的特征值与 $L_{\mathrm{c}}$ 在除一致性方向外的特征值相同，也就是 $\lambda_2(L_{\mathrm{c}}),\cdots,\lambda_N(L_{\mathrm{c}})$。

所以，$L_{\mathrm{c}}+J$ 的特征值为：

$$
1,\lambda_2(L_{\mathrm{c}}),\lambda_3(L_{\mathrm{c}}),\cdots,\lambda_N(L_{\mathrm{c}})
$$

因此：

$$
L_{\mathrm{c}}
+
\frac{1}{N}1_N1_N^{\mathrm{T}}
\succ0
$$

当且仅当：

$$
\lambda_2(L_{\mathrm{c}})>0
$$

也就是当且仅当通信图连通。

所以通信连通性约束可以等价写为：

$$
L_{\mathrm{c}}
+
\frac{1}{N}1_N1_N^{\mathrm{T}}
\succ0
$$

这个固定投影形式说明了为什么可以通过给一致性方向补上一个正特征值，来避开拉普拉斯矩阵固有的零特征值。但它还不是论文式 (21) 的原始写法；论文式 (21) 使用的是带辅助变量 $\xi$ 的矩阵不等式。

---

### 19.4 论文式 (21) 中的辅助变量 $\xi$

论文式 (21) 的核心矩阵不等式可以写成：

$$
\gamma I
\preceq
L_{\mathrm{c}}+\xi 1_N1_N^{\mathrm{T}}
$$

> 注：LMI 是 Linear Matrix Inequality，即线性矩阵不等式。它通常指一个关于优化变量仿射变化的对称矩阵被要求半正定，例如 $F(x)\succeq0$。这里的 $\gamma I\preceq L_{\mathrm{c}}+\xi1_N1_N^{\mathrm{T}}$ 等价于 $L_{\mathrm{c}}+\xi1_N1_N^{\mathrm{T}}-\gamma I\succeq0$，也就是要求该矩阵的所有特征值都不小于 $0$。这种形式比直接处理 $\lambda_2(L_{\mathrm{c}})$ 更适合放入半正定规划和后续分解求解框架。

其中，$\xi$ 是辅助实数变量。由于 $L_{\mathrm{c}}1_N=0$，而 $1_N1_N^{\mathrm{T}}$ 在 $1_N$ 方向上的特征值为 $N$，在与 $1_N$ 正交的子空间上为 $0$，所以：

$$
\lambda
\left(
L_{\mathrm{c}}+\xi 1_N1_N^{\mathrm{T}}
\right)
=
\left\{
\xi N,\lambda_2(L_{\mathrm{c}}),\lambda_3(L_{\mathrm{c}}),\cdots,\lambda_N(L_{\mathrm{c}})
\right\}
$$

矩阵不等式 $\gamma I\preceq L_{\mathrm{c}}+\xi 1_N1_N^{\mathrm{T}}$ 等价于要求该矩阵所有特征值都不小于 $\gamma$，因此：

$$
\gamma
\leq
\min
\left\{
\xi N,\lambda_2(L_{\mathrm{c}}),\lambda_3(L_{\mathrm{c}}),\cdots,\lambda_N(L_{\mathrm{c}})
\right\}
$$

又因为：

$$
\lambda_2(L_{\mathrm{c}})
\leq
\lambda_3(L_{\mathrm{c}})
\leq
\cdots
\leq
\lambda_N(L_{\mathrm{c}})
$$

所以上式可写成：

$$
\gamma
\leq
\min
\left\{
\xi N,\lambda_2(L_{\mathrm{c}})
\right\}
$$

由于 $\xi$ 不进入主要目标函数，也没有上限约束，优化中可以选取足够大的 $\xi$，使 $\xi N\geq\lambda_2(L_{\mathrm{c}})$。此时该不等式的有效约束就是：

$$
\gamma
\leq
\lambda_2(L_{\mathrm{c}})
$$

因此，式 (21) 第二项的作用不是固定地把 $1_N$ 方向的零特征值提升为 $1$，而是通过自由变量 $\xi$ 把一致性方向“抬高到足够大”，从而让矩阵不等式真正约束的是 $\lambda_2(L_{\mathrm{c}})$。

需要注意，这个 LMI 本身表达的是“$\gamma$ 是 $\lambda_2(L_{\mathrm{c}})$ 的一个下界”，即 $\gamma\leq\lambda_2(L_{\mathrm{c}})$。如果还希望用它保证通信图连通，就需要进一步要求 $\gamma>0$，或者在优化过程中推动 $\gamma$ 取正。否则，当 $\gamma=0$ 时，非连通图的 $\lambda_2(L_{\mathrm{c}})=0$ 也可能满足该不等式。

---

### 19.5 为什么原文又说 $\gamma$ 是 upper bound

论文在式 (21) 中还写了另一条约束：

$$
\lambda_2(L_{\mathrm{c}})
\leq
\gamma
$$

从这条约束本身看，$\gamma$ 确实是 $\lambda_2(L_{\mathrm{c}})$ 的 upper bound，即上界。

但上一小节已经说明，式 (21) 的第二个 LMI：

$$
\gamma I
\preceq
L_{\mathrm{c}}+\xi 1_N1_N^{\mathrm{T}}
$$

在 $\xi$ 可以自由选取时，实际给出的是：

$$
\gamma
\leq
\lambda_2(L_{\mathrm{c}})
$$

也就是说，两条约束的方向正好相反。合在一起，相当于把 $\gamma$ 夹在 $\lambda_2(L_{\mathrm{c}})$ 的两侧：

$$
\lambda_2(L_{\mathrm{c}})
\leq
\gamma
\leq
\lambda_2(L_{\mathrm{c}})
$$

因此得到：

$$
\gamma
=
\lambda_2(L_{\mathrm{c}})
$$

所以，从作者意图看，$\gamma$ 不是单纯为了给 $\lambda_2(L_{\mathrm{c}})$ 找一个松的上界，而是为了把难处理的谱量 $\lambda_2(L_{\mathrm{c}})$ 代理成一个优化变量。第一条约束给出 upper-bound 方向，第二个 LMI 给出 lower-bound 方向，两者共同试图把 $\gamma$ 绑定到代数连通度。

但从严格凸优化建模角度看，这里存在一个需要特别注意的问题：$\lambda_2(L_{\mathrm{c}})\leq\gamma$ 并不能像 $\lambda_2(L_{\mathrm{c}})\geq\gamma$ 那样自然写成标准凸 LMI。

> 注：这也可以从凸分析角度理解。对图拉普拉斯矩阵而言，$\lambda_2(L_{\mathrm{c}})$ 关于边权或关于由边权仿射生成的 $L_{\mathrm{c}}$ 是凹函数。凹函数的超水平集 $\{L_{\mathrm{c}}\mid \lambda_2(L_{\mathrm{c}})\geq\gamma\}$ 是凸集，因此可以对应到半正定约束；但凹函数的次水平集 $\{L_{\mathrm{c}}\mid \lambda_2(L_{\mathrm{c}})\leq\gamma\}$ 一般不是凸集，所以不能期望它自然写成标准凸 LMI。这正是 lower-bound 方向可处理、upper-bound 方向困难的根本原因。

例如，下界约束：

$$
\lambda_2(L_{\mathrm{c}})
\geq
\gamma
$$

可以写成：

$$
L_{\mathrm{c}}
\succeq
\gamma
\left(
I-\frac{1}{N}1_N1_N^{\mathrm{T}}
\right)
$$

或者等价地写成前面的移位形式：

$$
\gamma I
\preceq
L_{\mathrm{c}}+\xi1_N1_N^{\mathrm{T}}
$$

但是，如果试图用：

$$
L_{\mathrm{c}}
\preceq
\gamma
\left(
I-\frac{1}{N}1_N1_N^{\mathrm{T}}
\right)
$$

来表示 $\lambda_2(L_{\mathrm{c}})\leq\gamma$，则并不正确。这个约束实际限制的是 $L_{\mathrm{c}}$ 在 $1_N$ 正交子空间上的最大特征值：

$$
\lambda_N(L_{\mathrm{c}})
\leq
\gamma
$$

也就是控制拉普拉斯矩阵的最大非零特征值，而不是只控制第二小特征值。

因此，$\lambda_2(L_{\mathrm{c}})$ 的上界约束本身带有非凸性质。如果在算法实现中把 $\lambda_2$ 当作独立标量变量使用，就还需要额外机制保证这个标量等于真实矩阵 $L_{\mathrm{c}}$ 的第二小特征值；否则，这个 upper-bound 方向可能只是形式上的标量约束，无法真正反向限制通信拓扑。

所以，这里更稳妥的理解是：式 (21) 中的下界 LMI 是合理且有用的，它能够把“代数连通度至少达到某个正下界”写成半正定约束；但 upper-bound 方向在标准凸 LMI 意义下并不严密。原文通过 $\gamma$ 和 $\lambda_2$ 进行夹逼，更像是为了把难以直接处理的第二小特征值显式拿出来，变成后续优化中便于协调和约束的辅助量，而不是一个完全无漏洞的凸等价变换。

---

### 19.6 为什么这个变换有用

这个变换的价值在于，它把“代数连通度具有正下界”这样的谱约束，部分转化成了更容易处理的矩阵半正定约束。

原始约束：

$$
\lambda_2(L_{\mathrm{c}})>0
$$

直接涉及“第二小特征值”，在优化中不够方便。

式 (21) 中第二个半正定约束为：

$$
\gamma I
\preceq
L_{\mathrm{c}}+\xi 1_N1_N^{\mathrm{T}}
$$

它可以合理地给出：

$$
\gamma
\leq
\lambda_2(L_{\mathrm{c}})
$$

因此，当 $\gamma>0$ 时，它能够保证通信图连通。

由于：

$$
L_{\mathrm{c}}=\Lambda_{\mathrm{c}}-M_{\mathrm{c}}
$$

而 $\Lambda_{\mathrm{c}}$ 又由 $M_{\mathrm{c}}$ 的行和决定，所以 $L_{\mathrm{c}}$ 是 $M_{\mathrm{c}}$ 的仿射函数。因此，这个下界方向的半正定约束可以嵌入到后续的半正定规划或混合整数半正定规划框架中。

当然，由于本文中 $M_{\mathrm{c},ij}$ 仍然是 $0$-$1$ 变量，所以整个问题仍然具有组合优化特征；同时，式 (21) 中 $\lambda_2(L_{\mathrm{c}})\leq\gamma$ 这一 upper-bound 方向并不是标准凸 LMI，不能简单视为半正定约束。因此，式 (21) 更像是将代数连通度引入优化模型的启发式或松弛处理，而不是完全严密的凸等价变换。

---

### 19.7 当前理解小结

论文中的基本通信拓扑约束可以概括为：

$$
\begin{cases}
M_{\mathrm{c}}=M_{\mathrm{c}}^{\mathrm{T}},\\
M_{\mathrm{c},ii}=0,\\
M_{\mathrm{c},ij}\in\{0,1\},\\
\Lambda_{\mathrm{c},ii}=\sum_{j=1}^{N}M_{\mathrm{c},ij},\\
\Lambda_{\mathrm{c},ij}=0,\quad i\neq j,\\
L_{\mathrm{c}}=\Lambda_{\mathrm{c}}-M_{\mathrm{c}}.
\end{cases}
$$

它们分别对应无向通信、无自环、边变量二值、度矩阵定义和通信拉普拉斯矩阵定义。

其中最关键的是连通性约束。原始图论条件为：

$$
\lambda_2(L_{\mathrm{c}})>0
$$

即通信图的代数连通度大于零。为了便于优化，论文利用拉普拉斯矩阵固有零特征值对应 $1_N$ 方向这一性质，将其写成带辅助变量 $\xi$ 的 LMI：

$$
\gamma I
\preceq
L_{\mathrm{c}}+\xi 1_N1_N^{\mathrm{T}}
$$

式 (21) 中第二个半正定约束可以合理地给出 $\lambda_2(L_{\mathrm{c}})$ 的下界，并在 $\gamma>0$ 时保证通信连通性；但式 (21) 中 $\lambda_2(L_{\mathrm{c}})\leq\gamma$ 这一 upper-bound 方向并不是标准凸 LMI，不能简单视为半正定约束。

因此，式 (21) 更像是将代数连通度引入优化模型的启发式或松弛处理，而不是完全严密的凸等价变换。它的工程作用，是把难以直接处理的谱连通性信息显式变量化，使其能够和 $M_{\mathrm{c}}$、$\Lambda_{\mathrm{c}}$ 一起进入约束和后续求解框架。

## 20. 最终通信网络优化问题的理解

Section III-B 5) 的作用，是把前面三个目标和通信拓扑约束合并成一个最终优化模型。前面几节已经分别解释了 $f_{\mathrm{CR}}$、$f_{\mathrm{CS}}$、$f_{\mathrm{NF}}$ 和连通性约束，因此本节只保留最终模型的结构和逻辑关系。

最终目标函数可以概括为：

$$
\min_{M_{\mathrm{c}},\Lambda_{\mathrm{c}},\lambda_2,\gamma}
f_{\mathrm{Total}}
=
f_{\mathrm{CR}}
+
\mu_1 f_{\mathrm{CS}}
+
\mu_2 f_{\mathrm{NF}}
$$

其中：

$$
\begin{cases}
f_{\mathrm{CR}}
=
\displaystyle
\int_0^{\infty}
e^{2v_{\mathrm{spe}}t}
x^{\mathrm{T}}(t)
\left(
Q+C^{\mathrm{T}}RC
\right)
x(t)\,\mathrm{d}t,\\
f_{\mathrm{CS}}
=
\|\Lambda_{\mathrm{c}}\|_{\ell_1},\\
f_{\mathrm{NF}}
=
\|\Lambda_{\mathrm{c}}-\bar{\Lambda}_{\mathrm{c}}I\|_{\ell_1}.
\end{cases}
$$

这里 $\mu_1>0$ 和 $\mu_2>0$ 是权重系数，用来调节通信稀疏性和通信生存性在总目标中的重要程度。由于三个目标的量纲和数值尺度不同，$\mu_1$、$\mu_2$ 不只是偏好参数，也起到尺度协调作用。

---

### 20.1 优化变量的分工

最终模型中显式出现的主要变量为：

$$
M_{\mathrm{c}},\quad
\Lambda_{\mathrm{c}},\quad
\lambda_2,\quad
\gamma
$$

其中，$M_{\mathrm{c}}$ 直接决定通信边，并通过 $C=\mathcal{H}(M_{\mathrm{c}})$ 影响 $f_{\mathrm{CR}}$；$\Lambda_{\mathrm{c}}$ 是由 $M_{\mathrm{c}}$ 行和得到的度矩阵，主要进入 $f_{\mathrm{CS}}$ 和 $f_{\mathrm{NF}}$：

$$
\Lambda_{\mathrm{c},ii}
=
\sum_{j=1}^{N}
M_{\mathrm{c},ij}
$$

这也是后文采用 ADMM 变量分裂的直观基础：通信边结构和节点度分布虽然线性相关，但它们在不同目标项中承担的角色不同。$\lambda_2$ 和 $\gamma$ 则是为了把代数连通度相关信息显式放进优化模型。

---

### 20.2 为什么最终问题难解

最终模型难解，主要不是因为某一个目标复杂，而是因为几个困难叠加在一起：

1. $M_{\mathrm{c},ij}$ 是 $0$-$1$ 变量，带来组合优化性质；
2. $f_{\mathrm{CR}}$ 是闭环动态性能指标，依赖 $A_{\mathrm{cl}}=A-BC$ 和状态轨迹 $x(t)$；
3. $C=\mathcal{H}(M_{\mathrm{c}})$，使通信拓扑和闭环控制性能耦合；
4. 连通性涉及 $\lambda_2(L_{\mathrm{c}})$ 和半正定约束，其中式 (21) 的 upper-bound 方向还带有松弛性质；
5. $M_{\mathrm{c}}$ 与 $\Lambda_{\mathrm{c}}$ 线性相关，但分别主导不同目标项。

因此，论文将最终问题视为混合整数半正定规划问题，并在后文使用 ADMM 框架进行分解求解。

---

### 20.3 当前理解小结

Section III-B 5) 可以理解为把前面所有分析收束成一个拓扑选择问题：

$$
\boxed{
\text{在满足通信图可行性和连通性约束的前提下，寻找一个收敛快、边数少、节点度分布较均衡的通信拓扑。}
}
$$

其中，$f_{\mathrm{CR}}$ 对应控制性能，$f_{\mathrm{CS}}$ 对应通信成本，$f_{\mathrm{NF}}$ 对应节点度均衡和近似通信生存性。三者共同构成最终目标函数，$\mu_1$ 和 $\mu_2$ 决定这种折中关系。

## 21. ADMM 前的度矩阵约束：从邻接矩阵到可行度序列

Section IV 在正式给出 ADMM 迭代之前，先补充了一个关键问题：如果把邻接矩阵 $M_{\mathrm{c}}$ 和度矩阵 $\Lambda_{\mathrm{c}}$ 分开更新，那么 $\Lambda_{\mathrm{c}}$ 子问题不能只追求边数少、度数均衡，还必须保证得到的度序列有可能对应一个实际通信图。

因此，本节的重点不是重新展开最终目标函数，而是解释两个问题：

1. 为什么 $M_{\mathrm{c}}$ 和 $\Lambda_{\mathrm{c}}$ 可以作为不同变量块处理；
2. 为什么更新 $\Lambda_{\mathrm{c}}$ 时还要额外加入 Lemma 2 和 Lemma 3 的图论约束。

---

### 21.1 $M_{\mathrm{c}}$ 与 $\Lambda_{\mathrm{c}}$ 的分工

在最终问题中，$f_{\mathrm{CR}}$ 主要通过通信矩阵影响闭环动态：

$$
M_{\mathrm{c}}
\rightarrow
C
\rightarrow
A_{\mathrm{cl}}
\rightarrow
x(t)
\rightarrow
f_{\mathrm{CR}}.
$$

而 $f_{\mathrm{CS}}$ 和 $f_{\mathrm{NF}}$ 主要依赖度矩阵：

$$
f_{\mathrm{CS}}
=
\|\Lambda_{\mathrm{c}}\|_{\ell_1},
\qquad
f_{\mathrm{NF}}
=
\|\Lambda_{\mathrm{c}}-\bar{\Lambda}_{\mathrm{c}}I\|_{\ell_1}.
$$

二者并不是独立变量。对任意节点 $i$，有：

$$
\Lambda_{\mathrm{c},ii}
=
\sum_{j=1}^{N}
M_{\mathrm{c},ij}.
$$

也就是说，$\Lambda_{\mathrm{c}}$ 由 $M_{\mathrm{c}}$ 的行和决定。论文将这种关系抽象为线性映射：

$$
\varpi M_{\mathrm{c}}
=
\Lambda_{\mathrm{c}}.
$$

这里的 $\varpi$ 可以理解为“对邻接矩阵取行和并形成度矩阵”的线性算子。需要注意的是，这不是从 $\Lambda_{\mathrm{c}}$ 唯一恢复 $M_{\mathrm{c}}$ 的反算子；同一个度序列通常可以对应多个不同邻接矩阵。因此，后文的变量分解更像是把“具体边选择”和“节点度分布”分开处理，再通过一致性约束把二者协调起来。

---

### 21.2 为什么只优化 $\Lambda_{\mathrm{c}}$ 不够

$\Lambda_{\mathrm{c}}$ 只记录每个节点的度数，不记录边具体连向哪里。因此，仅凭度矩阵无法判断通信图是否连通。

例如，两个通信图可以具有相同度序列，但一个连通，另一个不连通。若 $\Lambda_{\mathrm{c}}$ 子问题只优化 $f_{\mathrm{CS}}$ 和 $f_{\mathrm{NF}}$，可能得到一个边数较少且度数较均衡的度分布，但这个度分布未必能对应合法简单图，更未必能对应连通图。

所以，论文在更新 $\Lambda_{\mathrm{c}}$ 时加入 Lemma 2 和 Lemma 3 对应的附加约束。它们的作用不是直接构造最终拓扑，而是提前排除明显不可实现或不可能连通的度序列。

---

### 21.3 Lemma 2：潜在连通的基本筛选

论文引用 [33，Y. C. Zhao, Y. S. Zhang, and L. Y. Miao, 2009, “The degree sequence of connected graphs and the number of lower degree vertices of connected plannar graphs”]，给出潜在连通度序列的基本条件。若：

$$
\Lambda_{\mathrm{c}}
=
\operatorname{diag}
\left(
\Lambda_{\mathrm{c},11},
\cdots,
\Lambda_{\mathrm{c},NN}
\right),
$$

则一个可能对应连通图的度序列至少应满足：

$$
\Lambda_{\mathrm{c},ii}
\geq
1,
\quad
\forall i,
$$

以及：

$$
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}
\geq
2(N-1).
$$

第一条排除孤立节点；第二条来自无向连通图至少需要 $N-1$ 条边，而无向图的总度数等于边数的两倍：

$$
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}
=
2|E_{\mathrm{c}}|
\geq
2(N-1).
$$

因此，Lemma 2 可以理解为对 $\Lambda_{\mathrm{c}}$ 的潜在连通性筛选。

> 注：文献 [33] 还讨论了平面图等更特殊图类下的度序列条件。例如，对 $N\geq 3$ 的简单连通平面图，由欧拉公式可得边数上界 $|E_{\mathrm{c}}|\leq 3N-6$，因此总度数还满足：
$$
\sum_{i=1}^{N}
\Lambda_{\mathrm{c},ii}
=
2|E_{\mathrm{c}}|
\leq
6N-12.
$$
但本文的通信网络优化并没有施加“平面嵌入”或“通信边在几何平面上不能交叉”的约束，而是按一般简单无向通信图处理。换句话说，它不是平面图约束下的拓扑优化问题，因此不能把 $6N-12$ 这类平面图上界直接加入本文模型。论文保留的是更一般的潜在连通下界，而不是平面图专用的总度数上界。

---

### 21.4 Lemma 3：度序列可图化约束

论文进一步引用 [34，P. Erdős and T. Gallai, 1959, “On maximal paths and circuits of graphs”]，使用 Erdős-Gallai 型条件判断一个非负整数度序列是否可以由简单图实现。

若度序列按非增顺序排列，则需要总度数为偶数，并且对每个：

$$
d
\in
\{1,2,\cdots,N\},
$$

满足：

$$
\sum_{i=1}^{d}
\Lambda_{\mathrm{c},ii}
\leq
d(d-1)
+
\sum_{i=d+1}^{N}
\min
\left\{
d,\Lambda_{\mathrm{c},ii}
\right\}.
$$

这个不等式的含义是：前 $d$ 个高度节点之间最多贡献 $d(d-1)$ 个度数计数，它们与其余节点之间的连接贡献又受到其余节点度数和 $d$ 的共同限制。如果前 $d$ 个节点要求的度数过大，而剩余节点无法提供足够连接对象，则该度序列不可能由简单图实现。

> 注：Erdős-Gallai 不等式的必要性可以这样理解。取度数最大的前 $d$ 个节点构成集合 $S$，其度数和 $\sum_{i=1}^{d}\Lambda_{\mathrm{c},ii}$ 来自两类边：一类是 $S$ 内部的边，最多有 $d(d-1)/2$ 条，在度数和中被计两次，所以最多贡献 $d(d-1)$；另一类是 $S$ 与剩余节点之间的边。对任意剩余节点 $i>d$，它最多只能连向 $S$ 中的 $d$ 个节点，同时也不能超过自身度数 $\Lambda_{\mathrm{c},ii}$，所以贡献上界为 $\min\{d,\Lambda_{\mathrm{c},ii}\}$。把这两部分相加，就得到：$$
\sum_{i=1}^{d}\Lambda_{\mathrm{c},ii}\leq d(d-1)+\sum_{i=d+1}^{N}\min\left\{d,\Lambda_{\mathrm{c},ii}\right\}.$$
这说明该不等式是“任何简单图度序列必须满足”的必要条件。充分性则是 Erdős-Gallai 定理较深的一面，通常可通过 Havel-Hakimi 归约或等价的交换边构造证明：若总度数为偶数且所有这些前缀不等式均成立，则可以逐步构造出一个简单图实现该度序列。

论文在式 (24)–(28) 中把这些要求整理为线性约束。除了总度数下界，还需要对度序列排序。论文式 (25) 写作：

$$
\Lambda_{\mathrm{c},ii}
\geq
\Lambda_{\mathrm{c},i-1,i-1},
\quad
i=2,\cdots,N.
$$

按常规定义，非增排列通常写作 $d_1\geq d_2\geq\cdots\geq d_N$。因此，这里的不等号方向可能与作者的索引排列方式有关，阅读时重点应放在它的目的：先固定度序列顺序，再使用 Erdős-Gallai 型判据。

对于主不等式中的：

$$
\min
\left\{
d,\Lambda_{\mathrm{c},ii}
\right\},
$$

论文引入辅助变量 $\beta_{d,i}$，并使用：

$$
\beta_{d,i}
\leq
d,
\qquad
\beta_{d,i}
\leq
\Lambda_{\mathrm{c},ii}.
$$

于是主约束可写成：

$$
\sum_{i=1}^{d}
\Lambda_{\mathrm{c},ii}
\leq
d(d-1)
+
\sum_{i=d+1}^{N}
\beta_{d,i},
\quad
d=1,\cdots,N-1.
$$

这样，原本涉及 $\min\{\cdot,\cdot\}$ 的度序列判据就被转化为线性约束。论文强调这些附加约束不会显著增加优化负担，原因也在这里：它们主要是线性不等式。

> 注：Lemma 3 中还包含“总度数为偶数”的条件。论文列出的式 (24)–(28) 没有单独把偶数条件写成一个线性约束；从实现角度看，这可能依赖 $M_{\mathrm{c}}$ 的 $0$-$1$ 对称邻接结构和 $\Lambda_{\mathrm{c}}$ 与 $M_{\mathrm{c}}$ 的一致性关系来保证。阅读时不宜把式 (24)–(28) 理解为完整替代所有图论条件，而应理解为对 $\Lambda_{\mathrm{c}}$ 子问题加入的一组可处理的图论筛选约束。

---

### 21.5 当前理解小结

第21节可以压缩成一句话：在进入 ADMM 之前，论文先说明 $M_{\mathrm{c}}$ 负责具体边和闭环性能，$\Lambda_{\mathrm{c}}$ 负责度分布和通信代价；二者通过线性行和关系保持一致，而 $\Lambda_{\mathrm{c}}$ 更新还需要 Lemma 2 和 Lemma 3 来避免产生不可图化或明显不可能连通的度序列。

因此，这一段的作用是为后文 ADMM 变量分裂铺垫约束结构，而不是再次解释 $f_{\mathrm{CR}}$、$f_{\mathrm{CS}}$、$f_{\mathrm{NF}}$ 的物理含义。

---

## 22. 上层 ADMM：按目标和变量块分解

Section IV-A 的作用，是把最终 MISDP 拆成两个相互协调的变量块。标准 ADMM 处理的是：

$$
\min_{x,z}
f(x)+g(z),
\quad
\text{s.t. }
Ax=z.
$$

> 注：ADMM 的一般过程可以理解为“分开优化，交替协调”。对上述问题，先构造增广拉格朗日函数：$$
L_{\rho}(x,z,y)=f(x)+g(z)+y^{\mathrm{T}}(Ax-z)+\frac{\rho}{2}\|Ax-z\|_{\ell_2}^{2}.$$
> 然后在第 $k+1$ 轮交替执行：$$
\begin{cases}
\displaystyle
x^{(k+1)}=\arg \min_x L_{\rho}\left(x,z^{(k)},y^{(k)}\right),\\
\displaystyle
z^{(k+1)}=\arg \min_z L_{\rho}\left(x^{(k+1)},z,y^{(k)}\right),\\
\displaystyle
y^{(k+1)}=y^{(k)}+\rho\left(Ax^{(k+1)}-z^{(k+1)}\right).
\end{cases} $$
> 其中，$x$ 更新主要处理 $f(x)$，$z$ 更新主要处理 $g(z)$，$y$ 更新则累积一致性残差 $Ax-z$。本文上层 ADMM 只是把 $x,z,A$ 分别替换为 $\Upsilon,\Omega,\psi$。

本文对应关系为：
$$
x\leftrightarrow \Upsilon,
\quad
z\leftrightarrow \Omega,
\quad
A\leftrightarrow \psi.
$$

---

### 22.1 变量分裂

论文将变量分成两组：

$$
\begin{aligned}
\Upsilon
&=
\operatorname{diag}
\left(
M_{\mathrm{c}},
\lambda_2,
\gamma,
\beta^{\Upsilon}
\right),\\
\Omega
&=
\operatorname{diag}
\left(
\Lambda_{\mathrm{c}},
\lambda_2^{\Omega},
\gamma^{\Omega},
\beta
\right).
\end{aligned}
$$

其中，$\Upsilon$ 侧主要负责具体邻接矩阵 $M_{\mathrm{c}}$ 和闭环性能 $f_{\mathrm{CR}}$；$\Omega$ 侧主要负责度矩阵 $\Lambda_{\mathrm{c}}$，以及 $f_{\mathrm{CS}}$、$f_{\mathrm{NF}}$ 和度序列约束。

两组变量通过线性一致性约束联系：

$$
\psi\Upsilon
=
\Omega,
\quad
\psi
=
\operatorname{diag}
\left(
\varpi,1,1,1
\right).
$$

这里 $\varpi$ 表示从 $M_{\mathrm{c}}$ 到 $\Lambda_{\mathrm{c}}$ 的行和映射，其余三个 $1$ 表示标量辅助变量在两侧应保持一致。

---

### 22.2 上层增广拉格朗日函数

论文式 (30) 可写为：

$$
\begin{aligned}
L_{\rho}
\left(
\Upsilon,\Omega,y
\right)
=&
f_{\mathrm{CR}}
\left(
\Upsilon
\right)
+
\mu_1
f_{\mathrm{CS}}
\left(
\Omega
\right)
+
\mu_2
f_{\mathrm{NF}}
\left(
\Omega
\right)\\
&+
y^{\mathrm{T}}
\left(
\psi\Upsilon-\Omega
\right)
+
\frac{\rho}{2}
\left\|
\psi\Upsilon-\Omega
\right\|_{\ell_2}^{2}.
\end{aligned}
$$

前两类项分别对应控制性能和通信拓扑指标；后两项用于惩罚 $\psi\Upsilon$ 与 $\Omega$ 的不一致。

---

### 22.3 上层三步迭代

上层第 $i+1$ 轮迭代为：

$$
\begin{cases}
\displaystyle
\Omega^{(i+1)}
=
\arg\min_{\Omega}
L_{\rho}
\left(
\Upsilon^{(i)},\Omega,y^{(i)}
\right),
\quad
\text{s.t. } (17),(24)\text{--}(28),\\[8pt]
\displaystyle
\Upsilon^{(i+1)}
=
\arg\min_{\Upsilon}
L_{\rho}
\left(
\Upsilon,\Omega^{(i+1)},y^{(i)}
\right),
\quad
\text{s.t. } (16),(19),(21),\\[8pt]
\displaystyle
y^{(i+1)}
=
y^{(i)}
+
\rho
\left(
\psi\Upsilon^{(i+1)}
-
\Omega^{(i+1)}
\right).
\end{cases}
$$

三步的分工可以概括为：

| 步骤 | 更新对象 | 主要作用 | 难点 |
|---|---|---|---|
| Step 1 | $\Omega$ | 优化度矩阵、通信成本和节点度均衡 | 凸目标 + 线性约束 |
| Step 2 | $\Upsilon$ | 优化闭环性能并满足拓扑约束 | $0$-$1$、LMI、$f_{\mathrm{CR}}$ 耦合 |
| Step 3 | $y$ | 累积一致性残差 | 显式加法 |

> 注：Step 1 中，真正需要数值求解的是 $\Lambda_{\mathrm{c}}$ 子问题；$\lambda_2^{\Omega}$、$\gamma^{\Omega}$、$\beta$ 这些标量没有进入 $f_{\mathrm{CS}}$ 和 $f_{\mathrm{NF}}$，主要由二次惩罚项和对偶变量拉回到 $\Upsilon$ 侧对应值附近。因此它们可以理解为一致性辅助变量。

> 对应的闭式迭代律推导如下：
以任意一个 $\Omega$ 侧辅助标量 $\Omega_k$ 为例，Step 1 中与它有关的项只有：$$
\min_{\Omega_k}\left\{-y_k^{(i)}\Omega_k+\frac{\rho}{2}\left(\psi_k\Upsilon_k^{(i)}-\Omega_k\right)^2\right\}.$$
对 $\Omega_k$ 求偏导并令其为零：$$
-y_k^{(i)}-\rho\left(\psi_k\Upsilon_k^{(i)}-\Omega_k\right)=0,$$
即可得到：$$
\Omega_k^{(i+1)}=\psi_k\Upsilon_k^{(i)}+\frac{1}{\rho}y_k^{(i)}.$$
由于 $\lambda_2^{\Omega}$、$\gamma^{\Omega}$ 和 $\beta$ 对应的 $\psi_k=1$，因此：$$
\begin{cases}
\displaystyle
\lambda_2^{\Omega,(i+1)}=\lambda_2^{(i)}+\dfrac{1}{\rho}y_{\lambda_2}^{(i)},\\[6pt]
\displaystyle
\gamma^{\Omega,(i+1)}=\gamma^{(i)}+\dfrac{1}{\rho}y_{\gamma}^{(i)},\\[6pt]
\displaystyle
\beta^{(i+1)}=\beta^{\Upsilon,(i)}+\dfrac{1}{\rho}y_{\beta}^{(i)}.
\end{cases} $$
若在 Step 2 中只看自由标量 $\beta^{\Upsilon}$，同理可得：$$
\beta^{\Upsilon,(i+1)}=\beta^{(i+1)}-\frac{1}{\rho} y_{\beta}^{(i)}.$$
这里正负号的差别来自增广拉格朗日中的一致性项 $y^{\mathrm{T}}(\psi\Upsilon-\Omega)$：更新 $\Omega$ 侧变量时对偶项贡献为 $-y^{\mathrm{T}}\Omega$，更新 $\Upsilon$ 侧变量时则为 $+y^{\mathrm{T}}\psi\Upsilon$。

---

### 22.4 为什么 Step 2 还要再分解

Step 2 的核心困难是：

$$
\Upsilon
=
\operatorname{diag}
\left(
M_{\mathrm{c}},
\lambda_2,
\gamma,
\beta^{\Upsilon}
\right)
$$

内部变量的约束性质差别很大。$M_{\mathrm{c}}$ 受 $0$-$1$ 邻接矩阵约束，$\gamma$ 和 $\lambda_2$ 又涉及式 (21) 的半正定/代数连通度约束；而 $f_{\mathrm{CR}}$ 本身还依赖闭环矩阵 $A_{\mathrm{cl}}=A-BC$。因此，Step 2 不能像 Step 1 那样直接化成简单凸子问题。

这就是论文引入下层 ADMM 的原因：把 Step 2 进一步拆成“无约束性能下降”和“约束投影”两个部分。

---

### 22.5 上层收敛判据

上层停止条件由两个残差决定：

$$
\begin{cases}
\displaystyle
\tau_1^{\mathrm{pri}}
\left(
i
\right)
=
\psi\Upsilon^{(i)}
-
\Omega^{(i)},\\[6pt]
\displaystyle
\tau_1^{\mathrm{dual}}
\left(
i
\right)
=
\rho
\left(
\Omega^{(i)}
-
\Omega^{(i-1)}
\right).
\end{cases}
$$

当：

$$
\left\|
\tau_1^{\mathrm{pri}}
\left(
i
\right)
\right\|_{\ell_2}^{2}
\leq
\epsilon_1^{\mathrm{pri}},
\quad
\left\|
\tau_1^{\mathrm{dual}}
\left(
i
\right)
\right\|_{\ell_2}^{2}
\leq
\epsilon_1^{\mathrm{dual}},
$$

即可认为上层变量已经基本一致，且迭代变化足够小。

---

## 23. 下层 ADMM：把性能下降和约束投影分开

下层 ADMM 用来求解上层 Step 2。它不是重新设计一个新目标，而是把 $\Upsilon$ 子问题再拆一层：$\Theta$ 负责无约束目标下降，$\Upsilon$ 负责满足原来的拓扑约束。

---

### 23.1 下层变量分裂

引入与 $\Upsilon$ 结构相同的变量：

$$
\Theta
=
\operatorname{diag}
\left(
M_{\mathrm{c}}^{\Theta},
\lambda_2^{\Theta},
\gamma^{\Theta},
\beta^{\Theta}
\right),
\quad
\Theta=\Upsilon.
$$

其中，$\Theta$ 侧承接 $f_{\mathrm{CR}}$ 和上层耦合项，便于做无约束梯度更新；$\Upsilon$ 侧保留约束 (16)、(19)、(21)，用于把结果拉回可行域。

---

### 23.2 下层增广拉格朗日函数

论文式 (38) 可写为：

$$
\begin{aligned}
L_{\rho,\vartheta}
\left(
\Theta,\Upsilon,u
\right)
=&
f_{\mathrm{CR}}
\left(
\Theta
\right)
+
\left(
y^{(i)}
\right)^{\mathrm{T}}
\left(
\psi\Theta-\Omega^{(i+1)}
\right)\\
&+
\frac{\rho}{2}
\left\|
\psi\Theta-\Omega^{(i+1)}
\right\|_{\ell_2}^{2}
+
u^{\mathrm{T}}
\left(
\Upsilon-\Theta
\right)
+
\frac{\vartheta}{2}
\left\|
\Upsilon-\Theta
\right\|_{\ell_2}^{2}.
\end{aligned}
$$

其中 $u$ 是下层对偶变量，$\vartheta$ 是下层惩罚参数。

---

### 23.3 下层三步迭代

在上层第 $i+1$ 轮内部，下层第 $j+1$ 轮为：

$$
\begin{cases}
\displaystyle
\Theta^{(j+1)}
=
\arg\min_{\Theta}
L_{\rho,\vartheta}
\left(
\Theta,\Upsilon^{(j)},u^{(j)}
\right),\\[8pt]
\displaystyle
\Upsilon^{(j+1)}
=
\arg\min_{\Upsilon}
L_{\rho,\vartheta}
\left(
\Theta^{(j+1)},\Upsilon,u^{(j)}
\right),
\quad
\text{s.t. } (16),(19),(21),\\[8pt]
\displaystyle
u^{(j+1)}
=
u^{(j)}
+
\vartheta
\left(
\Upsilon^{(j+1)}
-
\Theta^{(j+1)}
\right).
\end{cases}
$$

这里 L1 是无约束目标下降，L2 是带约束投影，L3 是对偶变量更新。

---

### 23.4 Step L1：$f_{\mathrm{CR}}$ 的梯度计算

给定当前 $\Theta$，定义：

$$
A_{\mathrm{cl},v}^{\Theta}
=
\left(
A+v_{\mathrm{spe}}I
\right)
-
BC^{\Theta}.
$$

论文通过两个矩阵方程计算梯度。首先解式 (41)：

$$
A_{\mathrm{cl},v}^{\Theta}T
+
T
\left(
A_{\mathrm{cl},v}^{\Theta}
\right)^{\mathrm{T}}
=
-\Gamma.
$$

再解式 (42)：

$$
\left(
A_{\mathrm{cl},v}^{\Theta}
\right)^{\mathrm{T}}P
+
P
A_{\mathrm{cl},v}^{\Theta}
=
-
\left(
Q+
\left(
C^{\Theta}
\right)^{\mathrm{T}}
RC^{\Theta}
\right).
$$

其中，$T$ 可理解为初始状态协方差沿闭环轨迹传播后的累计量；$P$ 是二次型积分代价对应的 Lyapunov 权矩阵。

由 [23，Gaeini et al., 2021, “Optimization of communication network topology in distributed control systems subject to prescribed decay rate”] 的推导，性能指标对反馈矩阵的梯度可由 Lyapunov 方程微分得到。

为简化记号，令：

$$
F=A_{\mathrm{cl},v}^{\Theta},\quad S=Q+\left(C^{\Theta}\right)^{\mathrm{T}}RC^{\Theta}.
$$

则 $P$ 满足：

$$
F^{\mathrm{T}}P+PF+S=0,\quad f_{\mathrm{CR}}=\operatorname{tr}(P\Gamma).
$$

对 $C^{\Theta}$ 作微小扰动，有 $\mathrm{d}F=-B\,\mathrm{d}C^{\Theta}$，且：

$$
\mathrm{d}S=\left(\mathrm{d}C^{\Theta}\right)^{\mathrm{T}}RC^{\Theta}+\left(C^{\Theta}\right)^{\mathrm{T}}R\,\mathrm{d}C^{\Theta}.
$$

对 $F^{\mathrm{T}}P+PF+S=0$ 求微分：

$$
F^{\mathrm{T}}\mathrm{d}P+\mathrm{d}P\,F+\left(\mathrm{d}F\right)^{\mathrm{T}}P+P\,\mathrm{d}F+\mathrm{d}S=0.
$$

另一方面，$T$ 满足 $FT+TF^{\mathrm{T}}=-\Gamma$。利用迹运算的循环性质，可将：

$$
\mathrm{d}f_{\mathrm{CR}}=\operatorname{tr}\left(\mathrm{d}P\,\Gamma\right)
$$

中的 $\mathrm{d}P$ 消去，得到：

$$
\mathrm{d}f_{\mathrm{CR}}=\operatorname{tr}\left(\left[\left(\mathrm{d}F\right)^{\mathrm{T}}P+P\,\mathrm{d}F+\mathrm{d}S\right]T\right).
$$

代入 $\mathrm{d}F=-B\,\mathrm{d}C^{\Theta}$ 和 $\mathrm{d}S$，整理为 $\mathrm{d}f_{\mathrm{CR}}=\operatorname{tr}\left(G^{\mathrm{T}}\mathrm{d}C^{\Theta}\right)$，可得：

$$
\frac{\partial f_{\mathrm{CR}}}
{\partial C^{\Theta}}
=
2
\left(
RC^{\Theta}
-
B^{\mathrm{T}}P
\right)
T.
$$

因此：

$$
\frac{\partial f_{\mathrm{CR}}}
{\partial \Theta}
=
2
\left(
RC^{\Theta}
-
B^{\mathrm{T}}P
\right)
T
\frac{\partial C^{\Theta}}
{\partial \Theta}.
$$

> 注：式 (41) 在形式上是 Lyapunov 型方程，可看作 Sylvester 方程的特例。$\Gamma$ 表示初始条件的协方差；论文指出最优通信网络不依赖具体初始条件，因此 $\Gamma$ 可取单位阵等方便的正定矩阵。

---

### 23.5 Step L2：约束投影

固定 $\Theta^{(j+1)}$ 后，L2 只需解：

$$
\Upsilon^{(j+1)}
=
\arg\min_{\Upsilon}
\left\{
\left(
u^{(j)}
\right)^{\mathrm{T}}
\left(
\Upsilon-\Theta^{(j+1)}
\right)
+
\frac{\vartheta}{2}
\left\|
\Upsilon-\Theta^{(j+1)}
\right\|_{\ell_2}^{2}
\right\},
\quad
\text{s.t. } (16),(19),(21).
$$

等价地说，它是在找一个满足约束的 $\Upsilon$，使其尽量接近：

$$
\Theta^{(j+1)}
-
\frac{1}{\vartheta}
u^{(j)}.
$$

因此 L2 可以理解为投影问题。它仍然包含 $M_{\mathrm{c}}$ 的 $0$-$1$ 约束和 LMI 约束；严格来看，这一步仍带有混合整数性质。若实际实现中希望用连续凸优化求解器处理，通常需要先把 $0$-$1$ 约束松弛为 $0\leq M_{\mathrm{c},ij}\leq 1$，将子问题近似转成 SDP/QP 类问题，再在最终阶段舍入回离散拓扑。

> 注：松弛加舍入是工程上常见处理，但理论上不能自动保证舍入后的拓扑仍满足所有连通性/LMI 条件；因此若舍入后不可行，需要检查连通性并进行修正。

---

### 23.6 下层收敛判据

下层残差与上层形式相同：

$$
\begin{cases}
\displaystyle
\tau_2^{\mathrm{pri}}
\left(
j
\right)
=
\Upsilon^{(j)}
-
\Theta^{(j)},\\[6pt]
\displaystyle
\tau_2^{\mathrm{dual}}
\left(
j
\right)
=
\vartheta
\left(
\Theta^{(j)}
-
\Theta^{(j-1)}
\right).
\end{cases}
$$

当：

$$
\left\|
\tau_2^{\mathrm{pri}}
\left(
j
\right)
\right\|_{\ell_2}^{2}
\leq
\epsilon_2^{\mathrm{pri}},
\quad
\left\|
\tau_2^{\mathrm{dual}}
\left(
j
\right)
\right\|_{\ell_2}^{2}
\leq
\epsilon_2^{\mathrm{dual}},
$$

则下层迭代停止，并把当前 $\Upsilon$ 返回给上层 Step 2。

> 注：原文式 (46)–(47) 的印刷文本疑似混入了上层变量。按下层 ADMM 结构，此处应检查 $\Upsilon$ 与 $\Theta$ 的一致性，以及 $\Theta$ 的相邻迭代变化。

---

### 23.7 当前理解小结

双层 ADMM 的逻辑可以概括为：

$$
\begin{cases}
\text{上层：拆 } \Omega \text{ 与 } \Upsilon,
\text{即拆度矩阵目标和邻接矩阵控制目标},\\
\text{下层：拆 } \Theta \text{ 与 } \Upsilon,
\text{即拆无约束性能下降和约束投影}.
\end{cases}
$$

因此，论文的优化方法并不是一次性求解原始 MISDP，而是通过两层变量分裂，把问题改写成“度矩阵凸优化 + 控制性能梯度更新 + 拓扑约束投影 + 对偶一致性修正”的迭代流程。

---

## 笔记参考文献

本笔记在阅读过程中引用了以下文献：

[23] N. Gaeini, A. Moradi Amani, M. Jalili, and X. Yu, "Optimization of communication network topology in distributed control systems subject to prescribed decay rate," *IEEE Trans. Cybern.*, vol. 51, no. 8, pp. 4277–4285, Aug. 2021.
—— 首次将带衰减率约束的通信拓扑优化与 LQR 灵敏度分析结合，给出了本文下层 ADMM 中 $f_{\mathrm{CR}}$ 关于反馈矩阵 $C$ 的梯度公式 (40)–(42)。

[27] J. Khazaei, D. H. Nguyen, and A. Khazaei, "Consensus-Based Demand Response of PMSG Wind Turbines With Distributed Energy Storage Considering Capability Curves," *IEEE Trans. Sustain. Energy*, vol. 11, no. 4, pp. 2315–2324, Oct. 2020.
—— 首次提出 ES 一致性变量 $E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}$，将功率分担与 SOC 协调统一到分布式一致性框架中（第 11 节）。

[28] B. D. O. Anderson and J. B. Moore, *Linear Optimal Control*. Englewood Cliffs, NJ, USA: Prentice-Hall, 1971.
—— 经典最优控制教材。Lyapunov 方程与二次型积分代价的关系（第 16.12 节），LQR 型指标的理解框架。

[33] —— Lemma 2 的出处，关于满足连通度序列的图存在性条件（第 21 节）。

[34] P. Erdős and T. Gallai, "On maximal paths and circuits of graphs," *Acta Math. Acad. Sci. Hung.*, vol. 10, no. 3–4, pp. 337–356, Sep. 1959.
—— Erdős-Gallai 定理的原始文献，给出度数序列可由简单图实现的充要条件。论文 Lemma 3 使用该条件约束 $\Lambda_c$ 子问题的可行域（第 21 节）。
