# 《Optimization of Communication Network for Distributed Control of Wind Farm Equipped With Energy Storages》阅读记录

## 0. 阅读说明

本文是一篇发表在 *IEEE Transactions on Sustainable Energy* 上的论文，主题是配置分布式储能的风电场分布式控制及通信网络优化。

阅读本文时，可以先从电机控制和电力电子控制背景出发理解 DFIG 与 ES 的建模方式，再逐步补充后续涉及的现代控制理论、图论、通信拓扑优化和 ADMM 求解框架等内容。

---

## 1. DFIG 与 ES 系统结构理解

本文研究的是配置分布式储能的风电场。每台风机采用 DFIG，并在 DFIG 的直流母线上通过 DC/DC 变换器接入一个 ES 单元。

DFIG 侧包括：

* RSC：rotor-side converter，转子侧变流器；
* GSC：grid-side converter，网侧变流器；
* ES：energy storage，通过 DC/DC 接入直流母线。

文中为了降低 GSC 电流和功率损耗，假设正常运行下 GSC 不提供无功，即：

$$
Q_{\mathrm{g}} = 0
$$

因此风机整体无功可以近似写成：

$$
Q_{\mathrm{w}} = Q_{\mathrm{d}} = Q_{\mathrm{s}}
$$

也就是说，本文中的无功调节主要由 DFIG 本体承担，而不是由 GSC 或 ES 承担。

---

## 2. Fig. 2 中 DFIG 功率控制环的整体理解

Fig. 2 可以理解为 DFIG 的有功、无功功率控制等效模型。它不是完整的 DFIG 电磁暂态模型，而是服务于风电场级分布式控制的简化功率动态模型。

其结构可以概括为：

$$
\text{功率外环 PI}
+
\text{转子电流内环一阶等效}
+
\text{电流到功率的静态映射}
+
\text{功率反馈滤波}
$$

其中：

* $Q_{\mathrm{d}}^{\mathrm{ref}}$ 是 DFIG 无功功率参考，由风电场级通信/一致性控制给出；
* $P_{\mathrm{d}}^{\mathrm{ref}}$ 是 DFIG 有功功率参考，由 MPPT 跟踪决定；
* $i_{\mathrm{dr}}$ 是转子 d 轴电流，主要影响无功；
* $i_{\mathrm{qr}}$ 是转子 q 轴电流，主要影响有功；
* $1/(sT_{\mathrm{ir}}+1)$ 是转子电流内环闭环动态的一阶等效；
* $1/(sT_{\mathrm{fr}}+1)$ 是有功/无功功率反馈滤波或测量延迟。

需要注意，图中的 $s_{\mathrm{g}}$ 是 slip ratio，即转差率；而 PI 控制器中的 $s$ 是拉普拉斯算子，二者不是同一个量。

---

## 3. DFIG 有功功率为什么与转差率有关

DFIG 与 IPMSM 的一个重要差别是：DFIG 的定子直接并网，因此定子电角频率由电网决定，记为 $\omega_{\mathrm{s}}$。转子机械速度相对于同步速度存在转差。

若 $s_{\mathrm{g}}$ 表示转差率，则有：

$$
p_{\mathrm{n}} \omega_{\mathrm{m}} = (1-s_{\mathrm{g}})\omega_{\mathrm{s}}
$$

其中，$p_{\mathrm{n}}$ 是极对数，$\omega_{\mathrm{m}}$ 是机械角速度。

在定子磁链定向下，DFIG 的电磁转矩近似与转子 q 轴电流成正比：

$$
T_{\mathrm{e}} = \frac{3}{2}p_{\mathrm{n}}\frac{L_{\mathrm{m}}}{L_{\mathrm{s}}}\psi_{\mathrm{s}} i_{\mathrm{qr}}
$$

机械功率为：

$$
P_{\mathrm{d}} = T_{\mathrm{e}} \omega_{\mathrm{m}}
$$

代入：

$$
\omega_{\mathrm{m}}=\frac{(1-s_{\mathrm{g}})\omega_{\mathrm{s}}}{p_{\mathrm{n}}}
$$

可得：

$$
P_{\mathrm{d}} = (1-s_{\mathrm{g}})\frac{3L_{\mathrm{m}}\omega_{\mathrm{s}}\psi_{\mathrm{s}}}{2L_{\mathrm{s}}}i_{\mathrm{qr}}
$$

因此，如果 $\omega_{\mathrm{s}}$ 使用电角速度，极对数 $p_{\mathrm{n}}$ 会在 $T_{\mathrm{e}}\omega_{\mathrm{m}}$ 中抵消。这也解释了为什么论文中的有功表达式没有显式写出 $p_{\mathrm{n}}$。

从电机控制角度理解，就是：

$$
i_{\mathrm{qr}} \rightarrow T_{\mathrm{e}}
$$

$$
T_{\mathrm{e}} \times \omega_{\mathrm{m}} \rightarrow P_{\mathrm{d}}
$$

而 DFIG 的机械速度与同步电角速度之间存在转差，所以最终有功功率表达式中会出现 $(1-s_{\mathrm{g}})$。

---

## 4. DFIG 无功功率中的负补偿项 $Q_{\mathrm{m}}$

Fig. 2 中的无功功率可以理解为：

$$
Q_{\mathrm{d}} = K_{\mathrm{Q}} i_{\mathrm{dr}} - Q_{\mathrm{m}}
$$

其中：

$$
K_{\mathrm{Q}} = \frac{3L_{\mathrm{m}}\omega_{\mathrm{s}}\psi_{\mathrm{s}}}{2L_{\mathrm{s}}}
$$

$$
Q_{\mathrm{m}} = \frac{3\omega_{\mathrm{s}}\psi_{\mathrm{s}}^2}{2L_{\mathrm{s}}}
$$

这里的 $Q_{\mathrm{m}}$ 是建立定子磁链所需的励磁无功。

采用定子磁链定向：

$$
\psi_{\mathrm{sd}}=\psi_{\mathrm{s}},\qquad \psi_{\mathrm{sq}}=0
$$

定子磁链方程为：

$$
\psi_{\mathrm{sd}}=L_{\mathrm{s}} i_{\mathrm{sd}}+L_{\mathrm{m}} i_{\mathrm{dr}}
$$

由此得到：

$$
i_{\mathrm{sd}}=\frac{\psi_{\mathrm{s}}-L_{\mathrm{m}} i_{\mathrm{dr}}}{L_{\mathrm{s}}}
$$

忽略定子电阻和磁链暂态项时，可近似认为：

$$
v_{\mathrm{sd}}\approx 0
$$

$$
v_{\mathrm{sq}}\approx \omega_{\mathrm{s}}\psi_{\mathrm{s}}
$$

由 dq 坐标下的无功表达式可知，无功中包含两部分：

1. 与 $i_{\mathrm{dr}}$ 有关的可控无功；
2. 与 $\psi_{\mathrm{s}}$ 有关的励磁无功需求。

按照论文中“DFIG 对外输出无功为正”的符号约定，整理后得到：

$$
Q_{\mathrm{d}} =
\frac{3L_{\mathrm{m}}\omega_{\mathrm{s}}\psi_{\mathrm{s}}}{2L_{\mathrm{s}}}i_{\mathrm{dr}}
-

\frac{3\omega_{\mathrm{s}}\psi_{\mathrm{s}}^2}{2L_{\mathrm{s}}}
$$

因此，当 $i_{\mathrm{dr}}=0$ 时：

$$
Q_{\mathrm{d}} = -Q_{\mathrm{m}}
$$

这说明 DFIG 会从电网吸收励磁无功。若希望 DFIG 对外无功为零，则需要：

$$
i_{\mathrm{dr}}=\frac{\psi_{\mathrm{s}}}{L_{\mathrm{m}}}
$$

因此，转子 d 轴电流 $i_{\mathrm{dr}}$ 的作用之一就是补偿异步机建立气隙磁场所需的励磁无功。

这与 IPMSM 有明显不同。IPMSM 中永磁体提供主磁链，而 DFIG 作为异步机，需要通过励磁建立磁链，因此无功表达式中天然包含一个励磁无功偏置项。

---

## 5. Fig. 2 中反馈一阶环节的含义

Fig. 2 反馈支路中的一阶环节为：

$$
\frac{1}{sT_{\mathrm{fr}}+1}
$$

它不是 DFIG 本体，也不是机械环节，而是功率测量反馈滤波器或测量延迟。

也就是说，功率外环实际比较的不是：

$$
Q_{\mathrm{d}}^{\mathrm{ref}}-Q_{\mathrm{d}}
$$

而是：

$$
Q_{\mathrm{d}}^{\mathrm{ref}}-Q_{\mathrm{d},\mathrm{f}}
$$

其中：

$$
Q_{\mathrm{d},\mathrm{f}}=\frac{1}{sT_{\mathrm{fr}}+1}Q_{\mathrm{d}}
$$

有功环同理：

$$
P_{\mathrm{d},\mathrm{f}}=\frac{1}{sT_{\mathrm{fr}}+1}P_{\mathrm{d}}
$$

这个环节用于模拟实际控制中功率测量不能瞬时获得，并滤除开关纹波、采样噪声和功率计算中的高频波动。

与 IPMSM 类比，它更像速度反馈或电流反馈中的低通滤波器，而不是机械对象 $1/(Js+B)$。

---

## 6. DFIG 控制结构与 IPMSM 控制结构的类比

可以作如下类比：

|IPMSM 控制              |DFIG RSC 控制                 |
|---------------------|---------------------------|
|速度外环 PI               |有功/无功功率外环 PI                |
|电流内环控制定子电流 $i_{\mathrm{d}},i_{\mathrm{q}}$  |电流内环控制转子电流 $i_{\mathrm{dr}},i_{\mathrm{qr}}$  |
|$i_{\mathrm{q}}$ 主要控制转矩          |$i_{\mathrm{qr}}$ 主要控制有功             |
|$i_{\mathrm{d}}$ 用于 MTPA、弱磁或磁链调节 |$i_{\mathrm{dr}}$ 主要控制无功/励磁          |
|机械对象为 $1/(Js+B)$      |功率对象为电流到 $P,Q$ 的静态映射加电流内环动态 |
|定子由逆变器供电              |定子直接并网，转子侧由 RSC 控制          |

因此，Fig. 2 的控制对象不是机械速度，而是并网 DFIG 的有功和无功功率。

---

## 7. Fig. 3 中 ES 功率控制模型理解

Fig. 3 中 ES 的控制结构可以按照与 Fig. 2 类似的方式理解：

$$
\text{功率外环 PI}
+
\text{DC/DC 电感电流内环一阶等效}
+
\text{功率反馈滤波}
$$

其中：

* $P_{\mathrm{e}}^{\mathrm{ch},\mathrm{ref}}<0$ 是充电功率参考；
* $P_{\mathrm{e}}^{\mathrm{dis},\mathrm{ref}}>0$ 是放电功率参考；
* $i_{\mathrm{L}}^{\mathrm{ref}}$ 是 DC/DC 电感电流参考；
* $1/(sT_{\mathrm{id}}+1)$ 是 DC/DC 电流内环闭环动态等效；
* $1/(sT_{\mathrm{fd}}+1)$ 是 ES 功率反馈滤波或测量延迟；
* $U_{\mathrm{e}}^{\mathrm{ch}}$ 和 $U_{\mathrm{e}}^{\mathrm{dis}}$ 是充放电状态下的 ES 电压。

从功率外环角度看，ES 的被控对象是：

$$
i_{\mathrm{L}}^{\mathrm{ref}} \rightarrow i_{\mathrm{L}} \rightarrow P_{\mathrm{e}}
$$

由于 DC 侧功率近似为：

$$
P_{\mathrm{e}} = U_{\mathrm{e}} i_{\mathrm{L}}
$$

因此功率外环看到的等效对象可以写成：

$$
G_{\mathrm{ES}}(s) \approx \frac{U_{\mathrm{e}}}{sT_{\mathrm{id}}+1}
$$

也就是说，ES 功率环控制的不是 SOC 本身，而是储能经 DC/DC 变换器输出或吸收的有功功率 $P_{\mathrm{e}}$。SOC 是上层一致性控制中的慢状态变量。

---

## 8. DFIG 无功功率约束

DFIG 的无功参考需要满足容量约束：

$$
|Q_{\mathrm{d},i}^{\mathrm{ref}}|
\leq
\sqrt{S_{\mathrm{d},\mathrm{N}}^2-P_{\mathrm{d},i}^2}
$$

该式来自视在功率关系：

$$
S^2=P^2+Q^2
$$

即：

$$
P_{\mathrm{d},i}^2+Q_{\mathrm{d},i}^2\leq S_{\mathrm{d},\mathrm{N}}^2
$$

因此，当前有功 $P_{\mathrm{d},i}$ 越大，剩余可用无功容量越小。

这与 IPMSM 中电流限幅圆有类似直觉：

$$
i_{\mathrm{d}}^2+i_{\mathrm{q}}^2\leq i_{\max}^2
$$

当一个方向的分量占用越多，另一个方向的可调空间就越小。

---

## 9. ES 充放电功率约束

ES 的功率约束同时受到 DC/DC 变换器容量和 GSC 剩余容量限制。

### 9.1 放电约束

放电时：

$$
P_{\mathrm{e}}^{\mathrm{dis},\mathrm{ref}}>0
$$

约束为：

$$
|P_{\mathrm{e},i}^{\mathrm{dis},\mathrm{ref}}|
\leq
\min
\left(
P_{\mathrm{DC/DC}}^{\mathrm{lim}},
P_{\mathrm{GSC}}^{\mathrm{lim}}-P_{\mathrm{r},i}
\right)
$$

含义是：

1. ES 放电功率不能超过 DC/DC 变换器额定功率；
2. ES 放电功率还不能使 GSC 的传输功率超过其容量限制。

放电时，ES 向直流母线送出功率，这部分功率通常需要通过 GSC 送到电网，因此会占用 GSC 容量。

### 9.2 充电约束

充电时：

$$
P_{\mathrm{e}}^{\mathrm{ch},\mathrm{ref}}<0
$$

约束为：

$$
|P_{\mathrm{e},i}^{\mathrm{ch},\mathrm{ref}}|
\leq
\min
\left(
P_{\mathrm{DC/DC}}^{\mathrm{lim}},
P_{\mathrm{GSC}}^{\mathrm{lim}}+P_{\mathrm{r},i}
\right)
$$

含义是：

1. 充电功率同样不能超过 DC/DC 变换器额定功率；
2. 充电状态下，ES 从直流母线吸收功率，其与转子侧功率在 GSC 功率负担上的方向关系不同，因此 GSC 约束项写成 $P_{\mathrm{GSC}}^{\mathrm{lim}}+P_{\mathrm{r},i}$。

最重要的理解是：

$$
\text{ES 的充放电能力}
=

\min(\text{DC/DC 容量},\text{GSC 剩余容量})
$$

---

## 10. 风电场级有功分配逻辑

本文中 DFIG 的有功参考 $P_{\mathrm{d}}^{\mathrm{ref}}$ 由 MPPT 决定。若 TSO 给出整个风电场有功指令 $P_{\mathrm{wf}}^{\mathrm{ref}}$，则 ES 需要承担的有功指令为：

$$
P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}
=

P_{\mathrm{wf}}^{\mathrm{ref}}

-\sum_{i=1}^{N}P_{\mathrm{d},i}
$$

风电场有功平衡为：

$$
\Delta P =
P_{\mathrm{wf}}^{\mathrm{ref}}
-

\sum_{i=1}^{N}
(P_{\mathrm{d},i}+P_{\mathrm{e},i})
=0
$$

> 这里 $\Delta P$ 式中再次出现 $\sum P_{\mathrm{d},i}$ 并不是重复计算 DFIG 有功，而是从风电场总有功平衡角度写出的实际功率偏差。由 $P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}=P_{\mathrm{wf}}^{\mathrm{ref}}-\sum_iP_{\mathrm{d},i}$ 可知，$\Delta P=P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}-\sum_iP_{\mathrm{e},i}$，因此一致性控制的目标是使所有 ES 的实际有功输出之和跟踪该剩余有功指令。

因此可以理解为：

$$
\text{DFIG 尽量运行在 MPPT}
$$

$$
\text{ES 负责补偿风电场有功偏差}
$$

这也是本文将 ES 加入 DFIG 风机直流母线的主要目的之一：在不显著牺牲风机 MPPT 的情况下，提高风电场有功调节能力和平滑能力。

---

## 11. ES 一致性变量的作用

在风电场级有功分配中，TSO 给出整个风电场的有功指令 $P_{\mathrm{wf}}^{\mathrm{ref}}$，而各台 DFIG 的有功功率 $P_{\mathrm{d},i}$ 主要由 MPPT 和风速决定。因此，所有 ES 需要共同承担的有功功率指令为：

$$
P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}
=

P_{\mathrm{wf}}^{\mathrm{ref}}

-\sum_{i=1}^{N}P_{\mathrm{d},i}
$$

也就是说，ES 的总任务是使：

$$
\sum_{i=1}^{N}P_{\mathrm{e},i}
=

P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}
$$

从风电场总有功平衡角度看，这已经足够保证：

$$
\Delta P
=

P_{\mathrm{wf}}^{\mathrm{ref}}

-\sum_{i=1}^{N}
(P_{\mathrm{d},i}+P_{\mathrm{e},i})
=0
$$

但是，这个总量关系只说明“所有 ES 合起来应该输出或吸收多少功率”，并没有说明“每一个 ES 应该承担多少”。例如，若总共需要 ES 放电 $0.6~\mathrm{pu}$，可以平均分配，也可以让某些 ES 多放电、某些 ES 少放电。满足总功率平衡的分配方式有无穷多种。

因此，论文需要进一步引入一个分配准则。这个准则就是 ES 一致性变量：

$$
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}
$$

并要求：

$$
E_{\mathrm{e},1}=E_{\mathrm{e},2}=\cdots=E_{\mathrm{e},N}
$$

其中，$P_{\mathrm{e},i}$ 是第 $i$ 个 ES 的有功功率，$S_{\mathrm{e},i}$ 是其 SOC，$K_1$ 和 $K_2$ 是权重系数。

> SOC（State of Charge，荷电状态）可以理解为储能单元当前剩余能量占额定容量的比例，通常取值为 $0$ 到 $1$ 或表示为百分比。它描述的是“还剩多少电”，而功率 $P_{\mathrm{e}}$ 描述的是“当前充放电有多快”。在本文中，若约定 $P_{\mathrm{e}}>0$ 表示放电，则 SOC 会下降；若 $P_{\mathrm{e}}<0$ 表示充电，则 SOC 会上升。引入 SOC 的意义在于，多个 ES 参与风电场有功调节时，不能只让它们平均分担功率，还应考虑各自剩余能量状态：高 SOC 的 ES 更适合多放电，低 SOC 的 ES 更适合少放电或多充电。因此，论文将 SOC 与 ES 功率共同构成一致性变量 $E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}$，使储能系统在补偿风电场有功偏差的同时，也能兼顾 SOC 均衡和储能运行健康。

---

### 11.1 为什么不能只让功率平均分配

一种最简单的想法是让所有 ES 平均分担功率，即：

$$
P_{\mathrm{e},1}=P_{\mathrm{e},2}=\cdots=P_{\mathrm{e},N}
$$

这样做可以实现功率平均分配，但问题是它完全忽略了 SOC 差异。

如果某个 ES 的 SOC 已经较低，而另一个 ES 的 SOC 较高，仍然让它们承担相同放电功率，就可能导致低 SOC 的 ES 进一步过度放电；反过来，在充电工况下，如果所有 ES 平均充电，也可能导致高 SOC 的 ES 更接近上限。

因此，单纯的功率平均分配并不一定有利于储能系统的长期健康状态。

---

### 11.2 为什么不能只让 SOC 一致

另一种想法是只让所有 ES 的 SOC 保持一致，即：

$$
S_{\mathrm{e},1}=S_{\mathrm{e},2}=\cdots=S_{\mathrm{e},N}
$$

但这也不够。因为风电场首先需要满足有功调度指令，也就是 ES 必须快速补偿：

$$
P_{\mathrm{wf}}^{\mathrm{ref}}
-\sum_{i=1}^{N}P_{\mathrm{d},i}
$$

SOC 是能量状态，变化速度通常比功率响应慢得多。如果只关注 SOC 一致性，可能无法保证 ES 的实际有功功率之和快速跟踪调度差额。

因此，ES 的分配目标需要同时考虑两个方面：

$$
\text{功率分担}
$$

和：

$$
\text{SOC 协调}
$$

这正是论文引入组合一致性变量 $E_{\mathrm{e},i}$ 的原因。

---

### 11.3 $E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}$ 的含义

论文定义：

$$
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}
$$

可以把 $E_{\mathrm{e},i}$ 理解成第 $i$ 个 ES 的“综合分担指标”。它不是一个直接的物理功率，也不是单纯的 SOC，而是把功率和 SOC 加权组合后得到的协调变量。

要求所有 ES 满足：

$$
E_{\mathrm{e},1}=E_{\mathrm{e},2}=\cdots=E_{\mathrm{e},N}
$$

含义是：各 ES 不一定要输出完全相同的功率，也不一定要求 SOC 在瞬间完全相同，而是让它们在“功率水平”和“SOC 状态”共同构成的指标上达到一致。

换句话说，论文不是简单地做平均功率分配，而是做一种兼顾 SOC 的功率分配。

---

### 11.4 从公式看它如何影响功率分配

假设所有 ES 的一致性变量最终收敛到同一个值 $E_{\mathrm{e}}^\ast$，则有：

$$
K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}=E_{\mathrm{e}}^\ast
$$

若 $K_1\neq 0$，可写为：

$$
P_{\mathrm{e},i}
=

\frac{E_{\mathrm{e}}^\ast}{K_1}

-\frac{K_2}{K_1}S_{\mathrm{e},i}
$$

这说明第 $i$ 个 ES 的功率分配不只取决于总功率需求，也会受到自身 SOC 的影响。

再结合总功率约束：

$$
\sum_{i=1}^{N}P_{\mathrm{e},i}
=
P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}
$$

可以得到类似如下的关系：

$$
P_{\mathrm{e},i}
=

\frac{P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}}{N}

-\frac{K_2}{K_1}
\left(
S_{\mathrm{e},i}-\bar S_{\mathrm{e}}
\right)
$$

其中：

$$
\bar S_{\mathrm{e}}=
\frac{1}{N}
\sum_{i=1}^{N}S_{\mathrm{e},i}
$$

这说明，ES 的功率分配可以看成两部分：

$$
\frac{P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}}{N}
$$

是平均功率分担项；

$$
-\frac{K_2}{K_1}
\left(
S_{\mathrm{e},i}-\bar S_{\mathrm{e}}
\right)
$$

是由 SOC 偏差引起的修正项。

因此，SOC 不同的 ES 不会被强制分配完全相同的功率，而是会根据 SOC 偏差进行调整。

---

### 11.5 关于 $K_1$ 和 $K_2$ 的理解

$K_1$ 和 $K_2$ 决定功率分担和 SOC 协调之间的权重。

如果 $K_2$ 的影响较小，则一致性变量主要由 $P_{\mathrm{e},i}$ 决定，此时控制效果更接近功率平均分配。

如果 $K_2$ 的影响较大，则 SOC 偏差对功率分配的影响更明显，系统会更重视不同 ES 之间的 SOC 协调。

需要注意，$K_1$ 和 $K_2$ 的符号和大小应与功率符号约定一致。本文中通常可理解为 $P_{\mathrm{e}}>0$ 表示放电，$P_{\mathrm{e}}<0$ 表示充电；在这种约定下，如果希望高 SOC 的 ES 在放电时承担更多功率、低 SOC 的 ES 在充电时获得更多补能，则 $K_2/K_1$ 的符号需要按该目标合理选取。论文这里只说明 $K_1$ 和 $K_2$ 是常数，并未在该处展开其具体整定方法。

因此，阅读时不应只把 $K_1$ 和 $K_2$ 理解为形式参数，而应理解为功率分担与 SOC 均衡之间的权衡系数。

---

### 11.6 一致性变量与分布式控制的关系

引入 $E_{\mathrm{e},i}$ 之后，每个 ES 不需要知道全场所有 ES 的状态，而只需要和通信邻居交换一致性变量或相关状态信息。典型的一致性控制思想是让每个节点根据邻居差异调整自身参考：

$$
E_{\mathrm{e},j}-E_{\mathrm{e},i}
$$

若某个 ES 的一致性变量与邻居不同，则通过调整 $P_{\mathrm{e},i}^{\mathrm{ref}}$ 使差异逐渐减小。最终所有 ES 的一致性变量趋于相同：

$$
E_{\mathrm{e},1}=E_{\mathrm{e},2}=\cdots=E_{\mathrm{e},N}
$$

同时，在 leader-follower 结构下，部分 leader 节点还会接收全场有功偏差 $\Delta P$，从而保证 ES 不只是内部达成一致，还能共同完成风电场总有功指令。

因此，ES 一致性变量的作用有两层：

$$
\text{局部层面：通过邻居通信实现分布式协调}
$$

$$
\text{全局层面：共同消除风电场有功偏差}
$$

---

### 11.7 物理意义总结

引入 ES 一致性变量的目的不是为了增加一个数学形式，而是为了回答“有功差额由所有 ES 分担时，具体该怎么分”的问题。

如果只考虑总功率平衡：

$$
\sum_{i=1}^{N}P_{\mathrm{e},i}
=

P_{\mathrm{wf},\mathrm{e}}^{\mathrm{ref}}
$$

那么分配方式不唯一。

如果只考虑功率平均，可能忽略 SOC，导致部分 ES 过度充放电。

如果只考虑 SOC 均衡，又可能无法快速满足风电场有功调度指令。

因此，论文引入：

$$
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}
$$

并要求：

$$
E_{\mathrm{e},1}=E_{\mathrm{e},2}=\cdots=E_{\mathrm{e},N}
$$

其本质是构造一个兼顾功率和 SOC 的分布式分配指标。

这一设计可以理解为：

$$
\boxed{
\text{ES 不只是平均分担有功功率，而是在满足总有功调度的同时，兼顾 SOC 均衡和储能健康状态。}
}
$$

因此，第 11 小节可以概括为：ES 一致性变量 $E_{\mathrm{e},i}$ 是风电场有功差额在多个分布式储能之间进行合理分配的协调指标，它把快速功率支撑和慢速 SOC 管理统一到了同一个分布式一致性控制框架中。

---

## 12. 风电场级无功分配逻辑

本文中，ES 只用于提供有功功率支撑，不参与无功调节。因此，当风电场无功输出与调度指令之间存在偏差时，该无功偏差只能由各台 DFIG 共同承担。

风电场级无功平衡关系为：

$$
Q_{\mathrm{wf},\mathrm{d}}^{\mathrm{ref}}=Q_{\mathrm{wf}}^{\mathrm{ref}}
$$

$$
\Delta Q=
Q_{\mathrm{wf}}^{\mathrm{ref}}
-

\sum_{i=1}^{N}Q_{\mathrm{d},i}

=0
$$

其中，$Q_{\mathrm{wf}}^{\mathrm{ref}}$ 是整个风电场的无功调度指令，$Q_{\mathrm{d},i}$ 是第 $i$ 台 DFIG 的无功输出。该式表示，所有 DFIG 的无功输出之和应跟踪风电场级无功指令。

与 ES 有功分配不同，DFIG 无功分配并不需要考虑类似 SOC 的能量状态。DFIG 提供无功主要受当前容量约束限制，即在当前有功输出 $P_{\mathrm{d},i}$ 下，其无功能力满足：

$$
|Q_{\mathrm{d},i}^{\mathrm{ref}}|
\leq
\sqrt{S_{\mathrm{d},\mathrm{N}}^2-P_{\mathrm{d},i}^2}
$$

因此，当前有功 $P_{\mathrm{d},i}$ 越大，该 DFIG 剩余可用于无功调节的容量越小。若简单要求所有 DFIG 输出相同无功，则可能导致部分无功能力较小的 DFIG 更早达到容量边界，而无功能力较大的 DFIG 没有充分利用。

为此，论文没有采用平均无功分配，而是引入 DFIG 的无功一致性变量：

$$
E_{\mathrm{d},i}=
\frac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}}
$$

其中，$A_{\mathrm{d},i}$ 表示第 $i$ 台 DFIG 的可调无功空间。论文要求：

$$
E_{\mathrm{d},1}=E_{\mathrm{d},2}=\cdots=E_{\mathrm{d},N}
$$

也就是：

$$
\frac{Q_{\mathrm{d},1}}{A_{\mathrm{d},1}}
=

\frac{Q_{\mathrm{d},2}}{A_{\mathrm{d},2}}

=\cdots

=\frac{Q_{\mathrm{d},N}}{A_{\mathrm{d},N}}
$$

该条件的含义是：各台 DFIG 不一定输出相同的无功功率，而是使其无功能力利用率保持一致。

如果所有 DFIG 的无功一致性变量最终收敛到同一个值 $\eta$，则有：

$$
Q_{\mathrm{d},i}=\eta A_{\mathrm{d},i}
$$

这说明第 $i$ 台 DFIG 分担的无功功率与其可调无功空间 $A_{\mathrm{d},i}$ 成正比。可调能力越大的 DFIG，承担的无功越多；可调能力越小的 DFIG，承担的无功越少。

论文中 $A_{\mathrm{d},i}$ 根据无功偏差方向分为两种情况：

$$
A_{\mathrm{d},i}
=

\begin{cases}
Q_{\mathrm{d},i,0}, & \Delta Q<0\\
\sqrt{S_{\mathrm{d},\mathrm{N}}^2-P_{\mathrm{d},i}^2}, & \Delta Q>0
\end{cases}
$$

当 $\Delta Q>0$ 时，说明风电场当前无功输出不足，需要增加无功输出。此时第 $i$ 台 DFIG 的可调无功空间主要由其容量圆决定：

$$
A_{\mathrm{d},i}
=

\sqrt{S_{\mathrm{d},\mathrm{N}}^2-P_{\mathrm{d},i}^2}
$$

也就是说，当前有功越小、剩余视在功率容量越大的 DFIG，能够承担更多增加无功的任务。

当 $\Delta Q<0$ 时，说明风电场当前无功输出偏多，需要减少无功输出。此时第 $i$ 台 DFIG 的可调空间与其原有无功输出 $Q_{\mathrm{d},i,0}$ 有关。原来输出无功较多的 DFIG，具有更大的向下调节空间，因此应承担更多减少无功的任务。

因此，DFIG 无功一致性变量 $E_{\mathrm{d},i}=Q_{\mathrm{d},i}/A_{\mathrm{d},i}$ 的作用不是让各台 DFIG 平均输出无功，而是让各台 DFIG 按照自身可调无功能力进行比例分担。

这与 ES 一致性变量有所不同。ES 的有功分配需要同时考虑功率输出和 SOC 状态，因此采用：

$$
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}
$$

而 DFIG 无功分配没有 SOC 这类能量状态，主要受瞬时容量约束限制，因此采用：

$$
E_{\mathrm{d},i}=\frac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}}
$$

可以理解为，ES 一致性变量强调“功率分担 + 能量状态协调”，而 DFIG 无功一致性变量强调“按无功能力比例分担”。

本小节可以总结为：

$$
\boxed{
\text{风电场无功偏差由 DFIG 承担，且各 DFIG 按可调无功空间进行比例分配。}
}
$$

$$
\boxed{
E_{\mathrm{d},i}=\frac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}}
\text{ 表示第 }i\text{ 台 DFIG 的无功能力利用率。}
}
$$

当所有 $E_{\mathrm{d},i}$ 达成一致时，各 DFIG 的无功能力利用率相同，从而避免简单平均分配导致部分机组过早达到无功容量边界。

---

## 13. 状态变量选取与 PI 积分状态的理解

在阅读本文由 Fig. 2 和 Fig. 3 推导状态空间模型时，需要先明确一点：论文中的 $Q_{\mathrm{d},\mathrm{int},i}$ 和 $P_{\mathrm{e},\mathrm{int},i}$ 中的 $\mathrm{int}$ 不是 initial，而是 integral，表示 PI 控制器中的积分状态。

也就是说：

$$
Q_{\mathrm{d},\mathrm{int},i}
=

\int
\left(
Q_{\mathrm{d},i}^{\mathrm{ref}}-Q_{\mathrm{d},i}
\right)\,\mathrm{d}t\\

P_{\mathrm{e},\mathrm{int},i}
=
\int
\left(
P_{\mathrm{e},i}^{\mathrm{ref}}-P_{\mathrm{e},i}
\right)\,\mathrm{d}t
$$

因此，这两个量不是初始值，而是控制器内部用于记录误差累计量的状态变量。它们本身在仿真或状态空间分析时当然也需要初始条件，例如 $Q_{\mathrm{d},\mathrm{int},i}(0)$ 和 $P_{\mathrm{e},\mathrm{int},i}(0)$，但其物理含义是“误差积分量”，而不是“初始值”。

---

### 13.1 为什么 PI 积分项需要作为状态变量

比例控制器没有记忆，其输出只由当前误差决定：

$$
u_{\mathrm{c}}=k_{\mathrm{p}}e
$$

但 PI 控制器含有积分项：

$$
u_{\mathrm{c}}=k_{\mathrm{p}}e+k_{\mathrm{i}}\int e(t)\,\mathrm{d}t
$$

此时，仅知道当前误差 $e(t)$ 并不能完全确定控制器输出，因为积分项还包含过去误差的累计效果。因此需要定义一个新的状态变量：

$$
z=\int e(t)\,\mathrm{d}t
$$

于是有：

$$
\dot z=e
$$

$$
u_{\mathrm{c}}=k_{\mathrm{p}}e+k_{\mathrm{i}}z
$$

这说明 PI 控制器本身是一个动态系统。为了把整个闭环系统写成标准状态空间形式：

$$
\dot x=f(x,u)
$$

必须把 PI 的积分状态 $z$ 放入状态向量。否则模型会隐含依赖过去误差的历史，系统状态就不封闭。

对应到本文中，DFIG 无功功率外环 PI 需要加入积分状态 $Q_{\mathrm{d},\mathrm{int},i}$，ES 有功功率外环 PI 需要加入积分状态 $P_{\mathrm{e},\mathrm{int},i}$。

---

### 13.2 本文单台 WT 状态变量的组成

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
\end{bmatrix}^{\mathrm{T}}
$$

这个状态向量可以分成两部分理解。

DFIG 无功通道对应：

$$
\begin{bmatrix}
Q_{\mathrm{d},i}&
Q_{\mathrm{d},\mathrm{int},i}&
i_{\mathrm{dr},i}
\end{bmatrix}^{\mathrm{T}}
$$

ES 有功通道对应：

$$
\begin{bmatrix}
P_{\mathrm{e},i}&
P_{\mathrm{e},\mathrm{int},i}&
i_{\mathrm{L},i}&
S_{\mathrm{e},i}
\end{bmatrix}^{\mathrm{T}}
$$

因此，本文状态变量的选取并不是随意列变量，而是把 Fig. 2 和 Fig. 3 中与通信一致性控制相关的动态环节都收进状态向量。

---

### 13.3 各状态变量的来源

$Q_{\mathrm{d},i}$ 是 DFIG 无功功率反馈量，也是无功一致性变量的一部分：

$$
E_{\mathrm{d},i}=\frac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}}
$$

同时，由于 Fig. 2 中存在功率反馈滤波环节，$Q_{\mathrm{d},i}$ 满足一阶动态关系：

$$
T_{\mathrm{fr}}\dot Q_{\mathrm{d},i}+Q_{\mathrm{d},i}=K_{\mathrm{Q}}i_{\mathrm{dr},i}
$$

其中：

$$
K_{\mathrm{Q}}=\frac{3L_{\mathrm{m}}\omega_{\mathrm{s}}\psi_{\mathrm{s}}}{2L_{\mathrm{s}}}
$$

所以 $Q_{\mathrm{d},i}$ 需要作为状态变量。

$Q_{\mathrm{d},\mathrm{int},i}$ 是 DFIG 无功外环 PI 的积分状态，用来记录无功误差的累计量，因此必须进入状态向量。

$i_{\mathrm{dr},i}$ 是 DFIG 转子 d 轴电流。Fig. 2 中转子电流内环被等效为一阶惯性环节：

$$
T_{\mathrm{ir}}\dot i_{\mathrm{dr},i}+i_{\mathrm{dr},i}=i_{\mathrm{dr},i}^{\mathrm{ref}}
$$

因此 $i_{\mathrm{dr},i}$ 也是动态状态。

$P_{\mathrm{e},i}$ 是 ES 的有功功率反馈量，同时进入 ES 一致性变量：

$$
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}
$$

Fig. 3 中 ES 功率反馈也包含一阶滤波动态：

$$
T_{\mathrm{fd}}\dot P_{\mathrm{e},i}+P_{\mathrm{e},i}=U_{\mathrm{e}}i_{\mathrm{L},i}
$$

因此 $P_{\mathrm{e},i}$ 需要作为状态变量。

$P_{\mathrm{e},\mathrm{int},i}$ 是 ES 有功外环 PI 的积分状态，用来记录 ES 有功功率误差的累计量。

$i_{\mathrm{L},i}$ 是 DC/DC 变换器电感电流。Fig. 3 中 DC/DC 电流内环被等效为：

$$
T_{\mathrm{id}}\dot i_{\mathrm{L},i}+i_{\mathrm{L},i}=i_{\mathrm{L},i}^{\mathrm{ref}}
$$

因此 $i_{\mathrm{L},i}$ 也需要进入状态向量。

$S_{\mathrm{e},i}$ 是 ES 的 SOC，属于储能单元的能量状态。论文中采用简化形式表示其动态：

$$
\dot S_{\mathrm{e},i}=-P_{\mathrm{e},i}
$$

同时，$S_{\mathrm{e},i}$ 还直接进入 ES 一致性变量 $E_{\mathrm{e},i}$，因此必须作为状态变量。

---

### 13.4 状态变量选取的一般规则

从控制理论角度看，状态变量的选取一般遵循以下原则：

1. 物理储能元件对应的变量通常需要作为状态，例如电感电流、电容电压、机械转速、SOC；
2. 一阶或高阶动态环节的输出通常需要作为状态，例如低通滤波器输出、测量滤波输出、闭环电流环等效输出；
3. 控制器内部动态需要作为状态，例如 PI 积分项、PLL 积分项、观测器状态；
4. 后续控制律或约束中显式使用的变量通常需要保留在状态向量中；
5. 状态变量应能使系统写成一阶微分方程组。

本文中的状态向量正好符合这一规则：

$$
\text{功率反馈滤波输出}
+
\text{PI 积分状态}
+
\text{电流内环状态}
+
\text{SOC 能量状态}
$$

---

### 13.5 为什么没有把 DFIG 有功环状态放进去

如果只从 Fig. 2(b) 出发，DFIG 有功环也可以写成类似状态：

$$
\begin{bmatrix}
P_{\mathrm{d},i}&
P_{\mathrm{d},\mathrm{int},i}&
i_{\mathrm{qr},i}
\end{bmatrix}^{\mathrm{T}}
$$

但本文后续通信一致性控制主要生成的是 $Q_{\mathrm{d},i}^{\mathrm{ref}}$ 和 $P_{\mathrm{e},i}^{\mathrm{ref}}$，而 DFIG 有功参考 $P_{\mathrm{d},i}^{\mathrm{ref}}$ 由 MPPT 决定：

$$
P_{\mathrm{d},i}^{\mathrm{ref}}=P_{\mathrm{MPPT},i}
$$

因此，$P_{\mathrm{d},i}$ 更多由风速、MPPT 策略和外部环境决定，而不是通信网络直接分配的控制变量。

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

但由于 $P_{\mathrm{d},i}^{\mathrm{ref}}$ 不由通信一致性控制生成，所以在用于通信拓扑优化的状态空间模型中，作者主要保留了 DFIG 无功通道和 ES 有功通道。

### 13.6 为什么可以写成式 (9) 的状态空间形式

前面已经说明，本文选择的状态变量来自 DFIG 无功功率反馈滤波、无功 PI 积分状态、转子 d 轴电流内环，以及 ES 有功功率反馈滤波、有功 PI 积分状态、DC/DC 电感电流内环和 SOC 能量状态。接下来的问题是：为什么这些变量可以进一步整理成论文式 (9) 的形式：

$$
\begin{cases}
\dot x_i=A_i x_i+B_i u_i \\

u_i=-\sum_{j\in\vartheta_i}C_{ij}x_j
\end{cases}
$$

这里的关键在于，Fig. 2 和 Fig. 3 中保留下来的动态环节本质上都是一阶线性环节或 PI 控制器积分环节。

功率反馈滤波、电流内环等效模型、PI 积分状态和 SOC 简化模型都可以写成“某个状态变量的导数等于当前状态变量和输入变量的线性组合”的形式。因此，在选定状态向量 $x_i$ 之后，每个状态变量的导数都可以由当前状态 $x_i$ 和外部输入 $u_i$ 表示。

在本文中，$u_i$ 不是底层变流器电压指令，而是风电场级一致性控制生成的功率参考变化率：

$$
u_i=
\begin{bmatrix}
\dot Q_{\mathrm{d},i}^{\mathrm{ref}}\\
\dot P_{\mathrm{e},i}^{\mathrm{ref}}
\end{bmatrix}
$$

也就是说，对于单台 WT 而言，底层 DFIG/ES 功率环可以看成一个线性动态系统，其输入是上层控制器给出的 $Q_{\mathrm{d},i}^{\mathrm{ref}}$ 和 $P_{\mathrm{e},i}^{\mathrm{ref}}$ 的变化率，其状态是 $Q_{\mathrm{d},i}$、$Q_{\mathrm{d},\mathrm{int},i}$、$i_{\mathrm{dr},i}$、$P_{\mathrm{e},i}$、$P_{\mathrm{e},\mathrm{int},i}$、$i_{\mathrm{L},i}$ 和 $S_{\mathrm{e},i}$。

因此，单台 WT 的底层模型可以统一写为：

$$
\dot x_i=A_i x_i+B_i u_i
$$

其中，$A_i$ 收集的是本机内部动态系数，例如功率滤波时间常数、电流内环时间常数、PI 参数以及电流到功率的静态增益；$B_i$ 则表示上层参考变化率对底层状态的作用方式。

进一步地，本文采用一致性控制来生成 $u_i$。一致性控制的基本思想是：第 $i$ 个节点根据自身与邻居节点之间的一致性变量差异来调整自己的参考值。如果某个节点与邻居不一致，就通过调整参考输入使差异逐渐减小。

对于 DFIG 无功分配，一致性变量是：

$$
E_{\mathrm{d},i}=\frac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}}
$$

对于 ES 有功分配，一致性变量是：

$$
E_{\mathrm{e},i}=K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}
$$

这两个一致性变量都是状态变量的线性组合。因此，基于邻居差异构造的控制律也可以写成状态变量的线性组合。

例如，典型一致性项具有如下形式：

$$
\sum_{j\in\vartheta_i}
(E_j-E_i)
$$

由于 $E_i$ 和 $E_j$ 都可以由对应节点的状态变量线性表示，所以该项最终可以整理成关于 $x_i$ 和邻居状态 $x_j$ 的线性表达式。

因此，上层输入 $u_i$ 可以写成：

$$
u_i=-\sum_{j\in\vartheta_i}C_{ij}x_j
$$

其中，$C_{ij}$ 不是底层物理参数矩阵，而是由一致性控制律、通信邻接关系、无功可调空间 $A_{\mathrm{d},i}$、ES 权重 $K_1,K_2$ 以及 leader-follower 结构等共同决定的反馈矩阵。

这里的负号体现了一致性控制的基本方向：当节点之间存在差异时，控制输入应推动差异减小，而不是扩大差异。

需要注意，式 (9) 并不是重新引入一个新的物理模型，而是把 Fig. 2、Fig. 3 中已经建立的底层线性动态模型和上层一致性控制律压缩到一个统一的状态空间表达式中。

其中：

$$
\dot x_i=A_i x_i+B_i u_i
$$

描述单台 WT/ES 的本地动态；

$$
u_i=-\sum_{j\in\vartheta_i}C_{ij}x_j
$$

描述通信网络和一致性控制如何生成本地参考变化率。

因此，式 (9) 的物理含义可以理解为：

$$
\boxed{
\text{本机底层功率环动态}
+
\text{基于邻居信息的一致性控制输入}
}
$$

在这个形式下，通信拓扑会通过邻居集合 $\vartheta_i$ 和矩阵 $C_{ij}$ 进入控制系统。后续将所有 WT 的状态堆叠起来后，就可以得到整个风电场的闭环系统矩阵，并进一步分析通信拓扑对收敛速度、连通性和鲁棒性的影响。

从建模角度看，本文能够写成式 (9)，依赖于以下几个前提：

1. 底层功率环被近似为线性一阶动态；
2. PI 控制器的积分项被显式加入状态变量；
3. 电流到功率的映射在当前工作点附近被视为线性关系；
4. 常值项和运行点偏置可以通过工作点平移或小信号建模吸收；
5. 一致性变量 $E_{\mathrm{d},i}$ 和 $E_{\mathrm{e},i}$ 是状态变量的线性组合；
6. 通信一致性控制律由邻居状态差构成，因此可以整理为状态反馈形式。

所以，式 (9) 可以看作是把“设备底层控制动态”和“风电场级分布式一致性控制”连接起来的中间模型。后续通信网络优化之所以能够转化为闭环矩阵和图论问题，正是因为该式把通信邻接关系转化成了状态反馈矩阵的一部分。

### 13.7 附录 A 中单台 WT 状态空间矩阵的来源

附录 A 给出的矩阵本质上是把 Fig. 2 和 Fig. 3 中的各个一阶动态环节逐行写成状态方程后，再按状态变量顺序整理得到的结果。

>严格说，论文式 (9) 将输入记为 $\dot Q_{\mathrm{d},i}^{\mathrm{ref}}$ 和 $\dot P_{\mathrm{e},i}^{\mathrm{ref}}$，但从 Fig. 2、Fig. 3 的 PI 框图及附录 A 中 $B_i$ 的结构看，$B_i$ 的各项更符合“参考值进入误差环节”的传统 PI 写法。因此本文笔记在解释 $A_i$、$B_i$ 的来源时，按 $Q_{\mathrm{d},i}^{\mathrm{ref}}$ 和 $P_{\mathrm{e},i}^{\mathrm{ref}}$ 进入底层 PI 环节来理解；而在解释一致性控制律和全场闭环矩阵时，再保留论文中“参考变化率由一致性控制生成”的记号。

---

#### 13.7.1 为什么 $A_i$ 是块对角矩阵

附录 A 中首先给出：

$$
A_i=
\begin{bmatrix}
A_{i,1}&0\\
0&A_{i,2}
\end{bmatrix}
$$

其中，$A_{i,1}$ 对应 DFIG 无功通道：

$$
\begin{bmatrix}
Q_{\mathrm{d},i}&
Q_{\mathrm{d},\mathrm{int},i}&
i_{\mathrm{dr},i}
\end{bmatrix}^{\mathrm{T}}
$$

$A_{i,2}$ 对应 ES 有功通道：

$$
\begin{bmatrix}
P_{\mathrm{e},i}&
P_{\mathrm{e},\mathrm{int},i}&
i_{\mathrm{L},i}&
S_{\mathrm{e},i}
\end{bmatrix}^{\mathrm{T}}
$$

矩阵采用块对角形式，说明作者在单台 WT 的底层动态建模中，把 DFIG 无功通道和 ES 有功通道视为两个相对独立的线性子系统。两者之间的协调关系不放在本机内部矩阵 $A_i$ 中，而是通过上层一致性控制输入 $u_i$ 体现。

---

#### 13.7.2 $A_{i,1}$ 的来源：DFIG 无功通道

附录 A 中的 DFIG 无功通道矩阵为：

$$
A_{i,1}
=
\begin{bmatrix}
-\frac{1}{T_{\mathrm{fr}}} & 0 & \frac{3L_{\mathrm{m}}\psi_{\mathrm{s}}\omega_{\mathrm{s}}}{2L_{\mathrm{s}}T_{\mathrm{fr}}}\\
-1 & 0 & 0\\
-\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}} & \frac{k_{\mathrm{r},\mathrm{i}}^{\mathrm{q}}}{T_{\mathrm{ir}}} & -\frac{1}{T_{\mathrm{ir}}}
\end{bmatrix}
$$

它的每一行都可以直接对应 Fig. 2(a) 中的一个动态关系。

第一行来自无功功率反馈滤波环节。DFIG 转子 d 轴电流 $i_{\mathrm{dr},i}$ 经过电流到无功的静态增益后，再经过功率测量滤波得到 $Q_{\mathrm{d},i}$，因此有：

$$
\dot Q_{\mathrm{d},i}
=
-\frac{1}{T_{\mathrm{fr}}}Q_{\mathrm{d},i}
+
\frac{3L_{\mathrm{m}}\psi_{\mathrm{s}}\omega_{\mathrm{s}}}{2L_{\mathrm{s}}T_{\mathrm{fr}}}i_{\mathrm{dr},i}
$$

所以 $A_{i,1}$ 第一行对应：

$$
\begin{bmatrix}
-\frac{1}{T_{\mathrm{fr}}}&
0&
\frac{3L_{\mathrm{m}}\psi_{\mathrm{s}}\omega_{\mathrm{s}}}{2L_{\mathrm{s}}T_{\mathrm{fr}}}
\end{bmatrix}
$$

第二行来自无功 PI 的积分状态。$Q_{\mathrm{d},\mathrm{int},i}$ 表示无功误差的积分量。若按 Fig. 2(a) 的传统 PI 控制框图理解，有：

$$
Q_{\mathrm{d},\mathrm{int},i}
=
\int
\left(
Q_{\mathrm{d},i}^{\mathrm{ref}}-Q_{\mathrm{d},i}
\right)\,\mathrm{d}t
$$

因此：

$$
\dot Q_{\mathrm{d},\mathrm{int},i}
=
Q_{\mathrm{d},i}^{\mathrm{ref}}
-
Q_{\mathrm{d},i}
$$

也就是说，$\dot Q_{\mathrm{d},\mathrm{int},i}$ 由两部分组成：一部分是状态项 $-Q_{\mathrm{d},i}$，另一部分是参考输入项 $Q_{\mathrm{d},i}^{\mathrm{ref}}$。

因此，在 $A_{i,1}$ 中，第二行只保留状态项：

$$
\begin{bmatrix}
-1&0&0
\end{bmatrix}
$$

而参考输入项 $Q_{\mathrm{d},i}^{\mathrm{ref}}$ 不进入 $A_{i,1}$，而是通过输入矩阵 $B_i$ 体现。由于该输入对 $\dot Q_{\mathrm{d},\mathrm{int},i}$ 的系数为 $1$，所以 $B_i$ 第一列第二行的元素为 $1$。

换言之，完整方程应理解为：

$$
\dot Q_{\mathrm{d},\mathrm{int},i}
=
\underbrace{
\begin{bmatrix}
-1&0&0
\end{bmatrix}
\begin{bmatrix}
Q_{\mathrm{d},i}\\
Q_{\mathrm{d},\mathrm{int},i}\\
i_{\mathrm{dr},i}
\end{bmatrix}
}_{A_{i,1}\text{ 中的状态项}}
+
\underbrace{1\cdot Q_{\mathrm{d},i}^{\mathrm{ref}}}_{B_i\text{ 中的输入项}}
$$

所以 $A_{i,1}$ 第二行写成 $\begin{bmatrix}-1&0&0\end{bmatrix}$，并不表示完整积分方程中只有 $-Q_{\mathrm{d},i}$，而是表示参考输入项已经被分离到 $B_i u_i$ 中。

第三行来自转子 d 轴电流内环的一阶等效。Fig. 2(a) 中，无功外环 PI 的输出是转子 d 轴电流参考 $i_{\mathrm{dr},i}^{\mathrm{ref}}$，电流内环则使实际转子 d 轴电流 $i_{\mathrm{dr},i}$ 跟踪该参考值。电流内环被简化为一阶惯性环节：

$$
T_{\mathrm{ir}}\dot i_{\mathrm{dr},i}+i_{\mathrm{dr},i}=i_{\mathrm{dr},i}^{\mathrm{ref}}
$$

无功外环 PI 的输出可以写成：

$$
i_{\mathrm{dr},i}^{\mathrm{ref}}
=

k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}
\left(
Q_{\mathrm{d},i}^{\mathrm{ref}}-Q_{\mathrm{d},i}
\right)
+
k_{\mathrm{r},\mathrm{i}}^{\mathrm{q}}Q_{\mathrm{d},\mathrm{int},i}
$$

将 PI 输出代入电流内环方程，可得：

$$
T_{\mathrm{ir}}\dot i_{\mathrm{dr},i}+i_{\mathrm{dr},i}
=

k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}
\left(
Q_{\mathrm{d},i}^{\mathrm{ref}}-Q_{\mathrm{d},i}
\right)
+
k_{\mathrm{r},\mathrm{i}}^{\mathrm{q}}Q_{\mathrm{d},\mathrm{int},i}
$$

整理为：

$$
\dot i_{\mathrm{dr},i}
=
-\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}Q_{\mathrm{d},i}
+
\frac{k_{\mathrm{r},\mathrm{i}}^{\mathrm{q}}}{T_{\mathrm{ir}}}Q_{\mathrm{d},\mathrm{int},i}
-
\frac{1}{T_{\mathrm{ir}}}i_{\mathrm{dr},i}
+
\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}Q_{\mathrm{d},i}^{\mathrm{ref}}
$$

因此，$\dot i_{\mathrm{dr},i}$ 中与状态变量 $Q_{\mathrm{d},i}$、$Q_{\mathrm{d},\mathrm{int},i}$ 和 $i_{\mathrm{dr},i}$ 相关的部分分别为：

$$
-\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}Q_{\mathrm{d},i}
$$

$$
\frac{k_{\mathrm{r},\mathrm{i}}^{\mathrm{q}}}{T_{\mathrm{ir}}}Q_{\mathrm{d},\mathrm{int},i}
$$

$$
-\frac{1}{T_{\mathrm{ir}}}i_{\mathrm{dr},i}
$$

所以在 $A_{i,1}$ 中，第三行对应：

$$
\begin{bmatrix}
-\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}&
\frac{k_{\mathrm{r},\mathrm{i}}^{\mathrm{q}}}{T_{\mathrm{ir}}}&
-\frac{1}{T_{\mathrm{ir}}}
\end{bmatrix}
$$

而参考输入项：

$$
\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}Q_{\mathrm{d},i}^{\mathrm{ref}}
$$

不属于 $A_{i,1}$，而是进入输入矩阵 $B_i$。因此，$B_i$ 第一列第三行的系数为：

$$
\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}
$$

换言之，$A_{i,1}$ 第三行表示本机状态对 $\dot i_{\mathrm{dr},i}$ 的影响，$B_i$ 第三行则表示无功参考输入通过 PI 比例环节对 $\dot i_{\mathrm{dr},i}$ 的影响。

因此，$A_{i,1}$ 可以理解为：

$$
\boxed{
\text{DFIG 无功功率滤波}
+
\text{无功 PI 积分状态}
+
\text{转子 d 轴电流内环}
}
$$

---

#### 13.7.3 $A_{i,2}$ 的来源：ES 有功通道

附录 A 中的 ES 有功通道矩阵为：

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

它对应 Fig. 3 中 ES 的有功功率控制环。按照状态变量顺序：

$$
x_{e,i}
=
\begin{bmatrix}
P_{\mathrm{e},i}\\
P_{\mathrm{e},\mathrm{int},i}\\
i_{\mathrm{L},i}\\
S_{\mathrm{e},i}
\end{bmatrix}
$$

其中，$P_{\mathrm{e},i}$ 是 ES 的实际有功功率，$P_{\mathrm{e},\mathrm{int},i}$ 是 ES 有功功率 PI 的误差积分项，$i_{\mathrm{L},i}$ 是 DC/DC 变换器电感电流，$S_{\mathrm{e},i}$ 是 ES 的 SOC。

第一行来自 ES 功率反馈滤波环节。Fig. 3 中，DC/DC 电感电流 $i_{\mathrm{L},i}$ 与 ES 端电压 $U_{\mathrm{e}}$ 相乘后得到 ES 功率，再经过功率反馈滤波得到反馈功率 $P_{\mathrm{e},i}$。因此可以写成：

$$
T_{\mathrm{fd}}\dot P_{\mathrm{e},i}+P_{\mathrm{e},i}=U_{\mathrm{e}} i_{\mathrm{L},i}
$$

整理得到：

$$
\dot P_{\mathrm{e},i}
=
-\frac{1}{T_{\mathrm{fd}}}P_{\mathrm{e},i}
+
\frac{U_{\mathrm{e}}}{T_{\mathrm{fd}}}i_{\mathrm{L},i}
$$

所以，在状态向量

$$
\begin{bmatrix}
P_{\mathrm{e},i}&
P_{\mathrm{e},\mathrm{int},i}&
i_{\mathrm{L},i}&
S_{\mathrm{e},i}
\end{bmatrix}^{\mathrm{T}}
$$

的顺序下，第一行对应：

$$
\begin{bmatrix}
-\frac{1}{T_{\mathrm{fd}}}&
0&
\frac{U_{\mathrm{e}}}{T_{\mathrm{fd}}}&
0
\end{bmatrix}
$$

这里的 $U_{\mathrm{e}}$ 可以理解为 ES 在当前充电或放电状态下的等效端电压。Fig. 3 中将充电和放电分别写成 $U_{\mathrm{e}}^{\mathrm{ch}}$ 和 $U_{\mathrm{e}}^{\mathrm{dis}}$，而在状态空间表达中，作者用统一的 $U_{\mathrm{e}}$ 表示对应工况下的电压系数。

第二行来自 ES 有功功率 PI 的积分状态。$P_{\mathrm{e},\mathrm{int},i}$ 表示 ES 有功功率误差的积分量。若按 Fig. 3 的传统 PI 控制框图理解，有：

$$
P_{\mathrm{e},\mathrm{int},i}
=
\int
\left(
P_{\mathrm{e},i}^{\mathrm{ref}}-P_{\mathrm{e},i}
\right)\,\mathrm{d}t
$$

因此：

$$
\dot P_{\mathrm{e},\mathrm{int},i}
=
P_{\mathrm{e},i}^{\mathrm{ref}}
-P_{\mathrm{e},i}
$$

也就是说，$\dot P_{\mathrm{e},\mathrm{int},i}$ 由两部分组成：一部分是状态项 $-P_{\mathrm{e},i}$，另一部分是参考输入项 $P_{\mathrm{e},i}^{\mathrm{ref}}$。

因此，在 $A_{i,2}$ 中，第二行只保留状态项：

$$
\begin{bmatrix}
-1&0&0&0
\end{bmatrix}
$$

而参考输入项 $P_{\mathrm{e},i}^{\mathrm{ref}}$ 不进入 $A_{i,2}$，而是通过输入矩阵 $B_i$ 体现。由于该输入对 $\dot P_{\mathrm{e},\mathrm{int},i}$ 的系数为 $1$，所以 $B_i$ 第二列第五行的元素为 $1$。

换言之，完整方程应理解为：

$$
\dot P_{\mathrm{e},\mathrm{int},i}
=
\underbrace{
\begin{bmatrix}
-1&0&0&0
\end{bmatrix}
\begin{bmatrix}
P_{\mathrm{e},i}\\
P_{\mathrm{e},\mathrm{int},i}\\
i_{\mathrm{L},i}\\
S_{\mathrm{e},i}
\end{bmatrix}
}_{A_{i}\text{ 中的状态项}}
+
\underbrace{1\cdot P_{\mathrm{e},i}^{\mathrm{ref}}}_{B_{i}\text{ 中的输入项}}
$$

所以 $A_{i,2}$ 第二行写成 $\begin{bmatrix}-1&0&0&0\end{bmatrix}$，并不表示完整积分方程中只有 $-P_{\mathrm{e},i}$，而是表示参考输入项已经被分离到 $B_i u_i$ 中。

第三行来自 DC/DC 电感电流内环的一阶等效。Fig. 3 中，ES 有功外环 PI 的输出是 DC/DC 电感电流参考 $i_{\mathrm{L},i}^{\mathrm{ref}}$，电流内环则使实际电感电流 $i_{\mathrm{L},i}$ 跟踪该参考值。电流内环被简化为一阶惯性环节：

$$
T_{\mathrm{id}}\dot i_{\mathrm{L},i}+i_{\mathrm{L},i}=i_{\mathrm{L},i}^{\mathrm{ref}}
$$

ES 有功外环 PI 的输出可以写成：

$$
i_{\mathrm{L},i}^{\mathrm{ref}}
=
k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}
\left(
P_{\mathrm{e},i}^{\mathrm{ref}}-P_{\mathrm{e},i}
\right)
+
k_{\mathrm{d},\mathrm{i}}^{\mathrm{p}}P_{\mathrm{e},\mathrm{int},i}
$$

将 PI 输出代入电流内环方程，可得：

$$
T_{\mathrm{id}}\dot i_{\mathrm{L},i}+i_{\mathrm{L},i}
=
k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}
\left(
P_{\mathrm{e},i}^{\mathrm{ref}}-P_{\mathrm{e},i}
\right)
+
k_{\mathrm{d},\mathrm{i}}^{\mathrm{p}}P_{\mathrm{e},\mathrm{int},i}
$$

整理为：

$$
\dot i_{\mathrm{L},i}
=

-\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}P_{\mathrm{e},i}
+
\frac{k_{\mathrm{d},\mathrm{i}}^{\mathrm{p}}}{T_{\mathrm{id}}}P_{\mathrm{e},\mathrm{int},i}
-

\frac{1}{T_{\mathrm{id}}}i_{\mathrm{L},i}
+
\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}P_{\mathrm{e},i}^{\mathrm{ref}}
$$

因此，$\dot i_{\mathrm{L},i}$ 中与状态变量 $P_{\mathrm{e},i}$、$P_{\mathrm{e},\mathrm{int},i}$ 和 $i_{\mathrm{L},i}$ 相关的部分分别为：

$$
-\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}P_{\mathrm{e},i}
$$

$$
\frac{k_{\mathrm{d},\mathrm{i}}^{\mathrm{p}}}{T_{\mathrm{id}}}P_{\mathrm{e},\mathrm{int},i}
$$

$$
-\frac{1}{T_{\mathrm{id}}}i_{\mathrm{L},i}
$$

所以在 $A_{i,2}$ 中，第三行对应：

$$
\begin{bmatrix}
-\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}&
\frac{k_{\mathrm{d},\mathrm{i}}^{\mathrm{p}}}{T_{\mathrm{id}}}&
-\frac{1}{T_{\mathrm{id}}}&
0
\end{bmatrix}
$$

而参考输入项：

$$
\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}P_{\mathrm{e},i}^{\mathrm{ref}}
$$

不属于 $A_{i,2}$，而是进入输入矩阵 $B_i$。因此，$B_i$ 第二列第六行的系数为：

$$
\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}
$$

换言之，$A_{i,2}$ 第三行表示本机状态对 $\dot i_{\mathrm{L},i}$ 的影响，$B_i$ 第六行则表示 ES 有功参考输入通过 PI 比例环节对 $\dot i_{\mathrm{L},i}$ 的影响。

第四行来自 SOC 动态。SOC 表示 ES 当前剩余能量占额定容量的比例。若令 $E_{\mathrm{N}}$ 表示 ES 的额定能量容量，并约定 $P_{\mathrm{e},i}>0$ 表示放电，则较一般的 SOC 动态可以写成：

$$
\dot S_{\mathrm{e},i}
=
-\frac{P_{\mathrm{e},i}}{E_{\mathrm{N}}}
$$

如果进一步考虑充放电效率，则可根据充电和放电工况分别加入效率系数。但本文主要关注风电场级分布式控制和通信拓扑优化，而不是电池内部精细建模，因此采用归一化形式，将容量基值吸收到功率标幺化中，写成：

$$
\dot S_{\mathrm{e},i}=-P_{\mathrm{e},i}
$$

这个式子的物理含义是：

* 当 $P_{\mathrm{e},i}>0$ 时，ES 放电，SOC 下降；
* 当 $P_{\mathrm{e},i}<0$ 时，ES 充电，SOC 上升。

由于 $\dot S_{\mathrm{e},i}$ 只与 $P_{\mathrm{e},i}$ 有关，因此在状态向量顺序

$$
\begin{bmatrix}
P_{\mathrm{e},i}&
P_{\mathrm{e},\mathrm{int},i}&
i_{\mathrm{L},i}&
S_{\mathrm{e},i}
\end{bmatrix}^{\mathrm{T}}
$$

下，第四行对应：

$$
\begin{bmatrix}
-1&0&0&0
\end{bmatrix}
$$

也就是说，$A_{i,2}$ 第四行表示：

$$
\dot S_{\mathrm{e},i}
=
\begin{bmatrix}
-1&0&0&0
\end{bmatrix}
\begin{bmatrix}
P_{\mathrm{e},i}\\
P_{\mathrm{e},\mathrm{int},i}\\
i_{\mathrm{L},i}\\
S_{\mathrm{e},i}
\end{bmatrix}
=-P_{\mathrm{e},i}
$$

这里没有对应的参考输入项，因此 $B_i$ 中 SOC 所在行的元素为 $0$。这也说明 SOC 不是底层功率环直接控制的快速变量，而是由 ES 实际充放电功率逐渐累积形成的慢状态。

因此，$A_{i,2}$ 可以整体理解为：

$$
\boxed{
\text{ES 有功功率反馈滤波}
+
\text{ES 有功 PI 积分状态}
+
\text{DC/DC 电感电流内环}
+
\text{SOC 能量状态}
}
$$

其中，前三行描述 ES 有功功率控制环的快速动态，第四行描述 ES 能量状态随充放电功率变化的慢动态。

---

#### 13.7.4 $B_i$ 的来源：参考指令输入

附录 A 中的 $B_i$ 是一个 $7\times 2$ 矩阵，用于描述上层功率参考输入如何注入到底层 DFIG 无功控制环和 ES 有功控制环中。

按照 7 维状态变量的顺序，$B_i$ 可以写成：

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

第一列对应 DFIG 无功参考输入对无功通道的作用。若按 Fig. 2(a) 的传统 PI 控制框图理解，无功误差积分状态满足：

$$
\dot Q_{\mathrm{d},\mathrm{int},i}
=
Q_{\mathrm{d},i}^{\mathrm{ref}}
-Q_{\mathrm{d},i}
$$

因此参考输入在第二行的系数为 $1$。同时，无功参考输入还会通过 PI 比例环节影响转子 d 轴电流参考 $i_{\mathrm{dr},i}^{\mathrm{ref}}$，再经过电流内环一阶等效作用于 $i_{\mathrm{dr},i}$，因此在第三行的系数为：

$$
\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}
$$

所以 $B_i$ 第一列为：

$$
\begin{bmatrix}
0\\
1\\
\frac{k_{\mathrm{r},\mathrm{p}}^{\mathrm{q}}}{T_{\mathrm{ir}}}\\
0\\
0\\
0\\
0
\end{bmatrix}
$$

第二列对应 ES 有功参考输入对 ES 有功通道的作用。同理，ES 有功误差积分状态满足：

$$
\dot P_{\mathrm{e},\mathrm{int},i}
=
P_{\mathrm{e},i}^{\mathrm{ref}}
-P_{\mathrm{e},i}
$$

因此参考输入在第五行的系数为 $1$。同时，有功参考输入通过 PI 比例环节影响 DC/DC 电感电流参考 $i_{\mathrm{L},i}^{\mathrm{ref}}$，再经过电流内环一阶等效作用于 $i_{\mathrm{L},i}$，因此在第六行的系数为：

$$
\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}
$$

所以 $B_i$ 第二列为：

$$
\begin{bmatrix}
0\\
0\\
0\\
0\\
1\\
\frac{k_{\mathrm{d},\mathrm{p}}^{\mathrm{p}}}{T_{\mathrm{id}}}\\
0
\end{bmatrix}
$$

因此，$B_i$ 的作用可以概括为：

$$
\boxed{
\text{将上层无功参考输入注入 DFIG 无功 PI 的积分项和比例项}
}
$$

$$
\boxed{
\text{将上层有功参考输入注入 ES 有功 PI 的积分项和比例项}
}
$$

需要注意，论文式 (9) 和附录中将输入记为参考值变化率 $\dot Q_{\mathrm{d},i}^{\mathrm{ref}}$、$\dot P_{\mathrm{e},i}^{\mathrm{ref}}$，而从 Fig. 2、Fig. 3 的 PI 框图及 $B_i$ 的结构来看，其作用形式更接近参考值 $Q_{\mathrm{d},i}^{\mathrm{ref}}$、$P_{\mathrm{e},i}^{\mathrm{ref}}$ 进入误差环节。因此，这里可以理解为作者为了衔接一致性控制律与状态空间模型而进行的记号压缩；阅读时应区分底层 PI 框图中的参考输入与上层一致性控制中生成的参考变化率。

---

#### 13.7.5 $C_{ij}$ 的来源：一致性控制律

附录 A 中的 $C_{ij}$ 并不是单台设备的底层物理参数矩阵，而是由多智能体一致性控制律整理得到的通信反馈矩阵。它描述第 $i$ 台 WT 如何根据自身状态、邻居状态以及全局功率偏差来调整自身的功率参考输入。

根据论文的 leader-follower 一致性控制思想，第 $i$ 台机组的上层输入可以理解为由两类信息构成：

1. 邻居间的一致性变量差异，用于实现 DFIG/ES 之间的分布式功率协调；
2. leader 节点接收的全局功率偏差，用于实现风电场总有功/无功指令跟踪。

其典型形式可以写成：

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

其中，$\vartheta_i$ 是第 $i$ 台 WT 的通信邻居集合，$M_{i0}=1$ 表示该节点为 leader，否则为 $0$。

DFIG 无功一致性变量为：

$$
E_{\mathrm{d},i}
=
\frac{Q_{\mathrm{d},i}}{A_{\mathrm{d},i}}
$$

ES 有功一致性变量为：

$$
E_{\mathrm{e},i}
=
K_1P_{\mathrm{e},i}+K_2S_{\mathrm{e},i}
$$

由于 $Q_{\mathrm{d},i}$、$P_{\mathrm{e},i}$ 和 $S_{\mathrm{e},i}$ 分别是状态向量 $x_i$ 的第 1、第 4 和第 7 个元素，因此一致性变量差可以直接展开成状态变量的线性组合：

$$
E_{\mathrm{d},i}-E_{\mathrm{d},j}
=
\frac{1}{A_{\mathrm{d},i}}Q_{\mathrm{d},i}
-\frac{1}{A_{\mathrm{d},j}}Q_{\mathrm{d},j}
$$

$$
E_{\mathrm{e},i}-E_{\mathrm{e},j}
=
K_1(P_{\mathrm{e},i}-P_{\mathrm{e},j})
+
K_2(S_{\mathrm{e},i}-S_{\mathrm{e},j})
$$

全局无功偏差为：

$$
\Delta Q
=
Q_{\mathrm{wf}}^{\mathrm{ref}}
-\sum_{k=1}^{N}Q_{\mathrm{d},k}
$$

全局有功偏差为：

$$
\Delta P
=

P_{\mathrm{wf}}^{\mathrm{ref}}

-\sum_{k=1}^{N}
\left(
P_{\mathrm{d},k}+P_{\mathrm{e},k}
\right)
$$

这里需要注意，$P_{\mathrm{d},k}$ 是 DFIG 在 MPPT 作用下的有功输出，它不是本文通信一致性控制直接调节的状态变量，但会作为外部运行点或扰动项进入风电场有功平衡。

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

从上述展开式可以看出，$\dot Q_{\mathrm{d},i}^{\mathrm{ref}}$ 与状态变量 $Q_{\mathrm{d}}$ 线性相关，$\dot P_{\mathrm{e},i}^{\mathrm{ref}}$ 与状态变量 $P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 线性相关。因此，当把全场所有 WT 的状态堆叠后，输入 $u$ 可以写成关于状态 $x$ 的线性或仿射表达式。

若忽略外部调度指令和 MPPT 有功项，或在某一工作点附近进行偏差变量建模，则常值项可以被吸收到平衡点中，此时可以写成紧凑形式：

$$
u=Cx
$$

或者结合论文式 (10) 的符号约定写成：

$$
\dot x
=Ax-Bu
=(A-BC)x
$$

因此，$C$ 或 $C_{ij}$ 的主要作用是从全场状态中提取 $Q_{\mathrm{d}}$、$P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 这些参与一致性控制的状态，并按照通信拓扑、控制增益和 leader-follower 结构进行加权组合。

从矩阵结构上看，$C_{ij}$ 在与 $Q_{\mathrm{d}}$、$P_{\mathrm{e}}$ 和 $S_{\mathrm{e}}$ 对应的列上具有非零元素，而在 PI 积分状态、电流内环状态等列上通常为零。这说明一致性控制直接使用的是无功输出、ES 有功功率和 SOC，而不是底层电流状态或 PI 积分状态。

因此，$C_{ij}$ 可以理解为：

$$
\boxed{
\text{通信邻居相对状态差所形成的拉普拉斯型反馈}
+
\text{leader 节点对全局功率偏差的跟踪反馈}
}
$$

它把以下因素统一吸收到矩阵表达中：

1. 一致性控制增益 $c_1$ 和 $c_2$；
2. DFIG 无功可调空间 $A_{\mathrm{d},i}$；
3. ES 功率/SOC 权重 $K_1$ 和 $K_2$；
4. 通信邻居集合 $\vartheta_i$ 或通信拉普拉斯矩阵 $L_{\mathrm{c}}$；
5. leader 标记 $M_{i0}$；
6. 全局偏差反馈增益 $c_0^{\mathrm{Q}}$ 和 $c_0^{\mathrm{P}}$。

因此，$C_{ij}$ 的本质不是设备物理模型，而是通信拓扑和一致性协议共同形成的状态反馈映射。它也是后续论文能够写出 $C=\mathcal{H}(M_{\mathrm{c}})$，并进一步研究通信网络拓扑如何影响闭环矩阵 $A_{\mathrm{cl}}=A-BC$ 的关键。

#### 13.7.6 当前理解小结

附录 A 中的矩阵可以按如下方式理解：

$$
A_i
=

\begin{bmatrix}
A_{i,1}&0\\
0&A_{i,2}
\end{bmatrix}
$$

其中，$A_{i,1}$ 来自 DFIG 无功功率环，$A_{i,2}$ 来自 ES 有功功率环。

$$
B_i
$$
描述上层输入：

$$
\dot Q_{\mathrm{d},i}^{\mathrm{ref}},\quad \dot P_{\mathrm{e},i}^{\mathrm{ref}}
$$

如何通过 PI 控制器和电流内环作用到本机状态。

$$
C_{ij}
$$

则来自一致性控制律，描述第 $i$ 个节点如何根据邻居节点 $j$ 的状态来调整自身参考变化率。

因此，附录 A 的矩阵并不是凭空给出的，而是由以下三部分逐步整理得到：

$$
\boxed{
\text{Fig. 2 的 DFIG 无功功率环}
}
$$

$$
\boxed{
\text{Fig. 3 的 ES 有功功率环}
}
$$

$$
\boxed{
\text{基于 }E_{\mathrm{d},i}\text{ 和 }E_{\mathrm{e},i}\text{ 的一致性控制律}
}
$$

其作用是把单台 WT 的底层功率控制和上层分布式一致性控制统一写成状态空间形式，为后续整个风电场闭环矩阵 $A_{\mathrm{cl}}=A-BC$ 的构造做准备。

### 13.8 当前理解小结

本文状态变量的选取可以概括为：

$$
\boxed{
\text{凡是具有动态记忆、参与反馈控制或进入一致性变量的量，都需要作为状态}
}
$$

其中，$Q_{\mathrm{d},\mathrm{int},i}$ 和 $P_{\mathrm{e},\mathrm{int},i}$ 是 PI 控制器的积分状态，不是初始值。它们之所以需要被记录，是因为 PI 控制器的输出不仅取决于当前误差，还取决于过去误差的累计量。

因此，本文单台 WT 的状态空间模型本质上是由以下部分组合而成：

$$
\boxed{
\text{DFIG 无功功率反馈滤波}
+
\text{DFIG 无功 PI 积分状态}
+
\text{DFIG 转子 d 轴电流内环}
}
$$

$$
\boxed{
\text{ES 有功功率反馈滤波}
+
\text{ES 有功 PI 积分状态}
+
\text{DC/DC 电感电流内环}
+
\text{SOC 能量状态}
}
$$

后续将所有 WT 的状态堆叠后，再通过一致性控制律把输入写成状态的线性组合，就可以得到整个风电场的闭环状态空间模型。

## 14. 图论基础：邻接矩阵、度矩阵、拉普拉斯矩阵与代数连通度

本文在 Section III-A 中首先给出了通信网络优化所需的图论基础。这里的图不是电气拓扑图，而是风电场中各 WT/ES 控制代理之间的通信网络图。

论文将通信网络表示为一个无向图：

$$
G=(V,E)
$$

其中，$V={v_1,v_2,\cdots,v_N}$ 是节点集合，每个节点可以理解为一台 WT/ES 控制代理；$E$ 是边集合，表示两个节点之间存在通信关系。

由于本文默认通信关系是双向的，即若节点 $i$ 能与节点 $j$ 通信，则节点 $j$ 也能与节点 $i$ 通信，因此该通信网络可以建模为无向图。

---

### 14.1 邻接矩阵

邻接矩阵记为：

$$
M=[M_{ij}]\in\mathbb{R}^{N\times N}
$$

其元素定义为：

$$
M_{ij}=
\begin{cases}
1, & v_i\text{ 与 }v_j\text{ 相连}\\
0, & v_i\text{ 与 }v_j\text{ 不相连}
\end{cases}
$$

由于本文采用无向通信图，因此有：

$$
M_{ij}=M_{ji}
$$

也就是邻接矩阵 $M$ 是对称矩阵。

需要注意，$M_{ij}$ 只有 $0$ 和 $1$，主要是因为本文使用的是无权图，而不是因为无向图本身必然只能取 $0$ 和 $1$。如果是加权无向图，邻接矩阵也可以取正权值；如果是有向图，邻接矩阵也可以是 $0$ 和 $1$，但一般不再对称。

---

### 14.2 度矩阵

节点 $v_i$ 的度表示与该节点直接相连的边数，也就是该节点的通信邻居数量。

对于无权图，第 $i$ 个节点的度可以写为：

$$
\Lambda_{ii}=\sum_{j=1}^{N}M_{ij}
$$

所有节点的度组成一个对角矩阵：

$$
\Lambda=
\mathrm{diag}
\left(
\Lambda_{11},
\Lambda_{22},
\cdots,
\Lambda_{NN}
\right)
$$

该矩阵称为度矩阵。

---

### 14.3 图拉普拉斯矩阵

图拉普拉斯矩阵定义为：

$$
L=\Lambda-M
$$

其中，$\Lambda$ 是度矩阵，$M$ 是邻接矩阵。

因此，$L$ 的元素具有如下结构：

$$
L_{ij}=
\begin{cases}
\Lambda_{ii}, & i=j\\
-1, & i\neq j\text{ 且 }v_i\text{ 与 }v_j\text{ 相连}\\
0, & i\neq j\text{ 且 }v_i\text{ 与 }v_j\text{ 不相连}
\end{cases}
$$

例如，对于 3 个节点的链式图：

$$
1-2-3
$$

邻接矩阵为：

$$
M=
\begin{bmatrix}
0&1&0\\
1&0&1\\
0&1&0
\end{bmatrix}
$$

度矩阵为：

$$
\Lambda=
\begin{bmatrix}
1&0&0\\
0&2&0\\
0&0&1
\end{bmatrix}
$$

因此拉普拉斯矩阵为：

$$
L=\Lambda-M
=

\begin{bmatrix}
1&-1&0\\
-1&2&-1\\
0&-1&1
\end{bmatrix}
$$

---

### 14.4 拉普拉斯矩阵是实对称半正定矩阵

对于本文中的无向无权通信图，拉普拉斯矩阵 $L$ 是实对称半正定矩阵。

首先，由于邻接矩阵 $M$ 是实矩阵，度矩阵 $\Lambda$ 也是实矩阵，因此：

$$
L=\Lambda-M
$$

也是实矩阵。

其次，由于无向图满足：

$$
M=M^{\mathrm{T}}
$$

而度矩阵 $\Lambda$ 是实对角矩阵，天然满足：

$$
\Lambda=\Lambda^{\mathrm{T}}
$$

因此：

$$
L^{\mathrm{T}}
=

(\Lambda-M)^{\mathrm{T}}

=\Lambda^{\mathrm{T}}-M^{\mathrm{T}}

=\Lambda-M

=L
$$

所以 $L$ 是实对称矩阵。

接下来证明 $L$ 半正定。对任意向量：

$$
x=
\begin{bmatrix}
x_1&x_2&\cdots&x_N
\end{bmatrix}^{\mathrm{T}}
$$

有：

$$
x^{\mathrm{T}}Lx=x^{\mathrm{T}}(\Lambda-M)x
$$

即：

$$
x^{\mathrm{T}}Lx
=

\sum_{i=1}^{N}\Lambda_{ii}x_i^2

-\sum_{i=1}^{N}\sum_{j=1}^{N}M_{ij}x_ix_j
$$

由于：

$$
\Lambda_{ii}=\sum_{j=1}^{N}M_{ij}
$$

所以：

$$
\sum_{i=1}^{N}\Lambda_{ii}x_i^2
=

\sum_{i=1}^{N}\sum_{j=1}^{N}M_{ij}x_i^2
$$

进一步可以得到：

$$
x^{\mathrm{T}}Lx
=

\frac{1}{2}
\sum_{i=1}^{N}
\sum_{j=1}^{N}
M_{ij}(x_i-x_j)^2
$$

由于 $M_{ij}\geq 0$，且 $(x_i-x_j)^2\geq 0$，因此：

$$
x^{\mathrm{T}}Lx\geq 0
$$

对任意 $x$ 都成立，所以 $L$ 是半正定矩阵。

因此，对于无向图的拉普拉斯矩阵，有：

$$
L=L^{\mathrm{T}}
$$

且：

$$
x^{\mathrm{T}}Lx\geq 0,\quad \forall x
$$

所以 $L$ 是实对称半正定矩阵。

---

### 14.5 为什么拉普拉斯矩阵一定有一个零特征值

图拉普拉斯矩阵还有一个重要性质：

$$
L\mathbf{1}=0
$$

其中：

$$
\mathbf{1}=
\begin{bmatrix}
1&1&\cdots&1
\end{bmatrix}^{\mathrm{T}}
$$

这是因为 $L$ 的每一行元素之和都为零。具体来说，拉普拉斯矩阵第 $i$ 行的对角元素是节点 $i$ 的度，而非对角元素中与邻居相连的位置为 $-1$，因此该行求和为：

$$
\Lambda_{ii}-\sum_{j=1}^{N}M_{ij}=0
$$

所以：

$$
L\mathbf{1}=0
$$

这说明 $0$ 一定是 $L$ 的一个特征值，对应的特征向量是全 1 向量 $\mathbf{1}$。

从一致性控制角度看，$\mathbf{1}$ 方向对应所有节点状态完全相同的情况：

$$
x_1=x_2=\cdots=x_N
$$

这正是系统已经达成一致的状态。

---

### 14.6 代数连通度与 Fiedler 值

由于无向图的拉普拉斯矩阵 $L$ 是实对称半正定矩阵，因此它的特征值均为实数，且可以按从小到大排列为：

$$
0=\lambda_1(L)\leq \lambda_2(L)\leq \cdots \leq \lambda_N(L)
$$

其中，最小特征值 $\lambda_1(L)$ 总是 $0$。真正用于衡量图是否连通的是第二小特征值：

$$
\lambda_2(L)
$$

该值称为图的代数连通度，也常称为 Fiedler 值。

其基本性质是：

$$
\lambda_2(L)>0
$$

当且仅当图 $G$ 是连通图。

如果图不连通，则至少可以分成两个互不通信的连通分量，此时拉普拉斯矩阵会有多个零特征值，因此：

$$
\lambda_2(L)=0
$$

更一般地说，如果图有 $k$ 个连通分量，那么拉普拉斯矩阵 $L$ 的零特征值重数就是 $k$。

因此，Fiedler 值不仅可以判断图是否连通，还可以在一定程度上反映网络的连通强度。

---

### 14.7 代数连通度的直观理解

如果一个通信图虽然连通，但只有一条关键边连接两个大区域，那么信息从一个区域传播到另一个区域时会受到限制。这种图的 Fiedler 值通常较小。

如果一个通信图有更多冗余连接和交叉连接，节点之间的信息扩散路径更多，则 Fiedler 值通常较大。

因此可以粗略理解为：

$$
\lambda_2(L)\text{ 越大，通信网络越“紧密”}
$$

在一致性控制中，典型动态可以写成：

$$
\dot x=-Lx
$$

该系统最终会收敛到一致状态。由于 $\lambda_1(L)=0$ 对应最终一致方向，不代表误差衰减；真正影响最慢一致性误差衰减速度的是：

$$
\lambda_2(L)
$$

所以在许多一致性控制问题中，$\lambda_2(L)$ 越大，信息扩散越快，一致性收敛速度也通常越快。

---

### 14.8 当前理解小结

这一小节的图论基础可以总结为：

$$
\boxed{
M\text{ 描述节点之间是否直接通信}
}
$$

$$
\boxed{
\Lambda\text{ 描述每个节点有多少通信邻居}
}
$$

$$
\boxed{
L=\Lambda-M\text{ 把通信拓扑转化为适合一致性分析的矩阵}
}
$$

$$
\boxed{
\lambda_2(L)\text{ 是 Fiedler 值，也就是代数连通度}
}
$$

其中，$L$ 是实对称半正定矩阵，且一定满足：

$$
L\mathbf{1}=0
$$

因此 $0$ 总是 $L$ 的一个特征值。第二小特征值 $\lambda_2(L)$ 用于判断图是否连通：

$$
\lambda_2(L)>0
\Longleftrightarrow
G\text{ 连通}
$$

在本文中，$\lambda_2(L_{\mathrm{c}})>0$ 是通信网络可用于全局一致性控制的基本前提。
