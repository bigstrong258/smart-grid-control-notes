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

因此，论文需要进一步引入一个分配准则。这个准则就是 ES 一致性变量 [27，Khazaei et al., 2020, “Consensus-Based Demand Response of PMSG Wind Turbines With Distributed Energy Storage Considering Capability Curves”]：

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
