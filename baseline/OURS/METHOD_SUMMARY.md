# 多机器人分布式轨迹规划方法完整总结

本文档详细总结了我们的三篇论文中实现的多机器人分布式轨迹规划方法，包括所有的数学公式和方法细节。

---

# 第一章：自由空间中的MPC多机器人轨迹生成

## 1.1 问题定义

### 1.1.1 机器人动力学

考虑 $N$ 个机器人组成的团队，每个机器人 $i \in \mathcal{N} = \{1, 2, \ldots, N\}$ 建模为 $\mathbb{R}^d$ 中的点质量（$d=2,3$），运动由双积分器近似：

$$x_i(t+h) = A x_i(t) + B u_i(t)$$

其中：
- $x_i(t) = [p_i(t), v_i(t)]^T$ 是机器人 $i$ 的状态，包含位置 $p_i(t)$ 和速度 $v_i(t)$
- $u_i(t)$ 作为加速度是控制输入
- 系统矩阵：
$$A = \begin{bmatrix} I_d & hI_d \\ 0_d & I_d \end{bmatrix}, \quad B = \begin{bmatrix} 0_d \\ hI_d \end{bmatrix}$$
- $h$ 是采样间隔

**物理约束**：
$$\|v_i\|_v \leq v_{max}, \quad \|u_i\|_a \leq a_{max}$$

其中 $v, a$ 是正定矩阵，$v_{max}, a_{max} > 0$ 分别表示最大速度和加速度。

### 1.1.2 碰撞避免约束

任意一对机器人之间的最小允许距离为 $r_{min} > 0$：
$$\|p_{ij}\| = \|p_i - p_j\| \geq r_{min}, \quad \forall (i,j) \in \mathcal{N} \times \mathcal{N}, i \neq j$$

### 1.1.3 MPC问题描述

在时刻 $t \geq t_0$，机器人 $i$ 的规划轨迹定义为：
$$P_i = [p_i^1, p_i^2, \ldots, p_i^K]$$

其中 $p_i^k$ 是时刻 $t+kh$ 的规划位置，$k \in \mathcal{K} = \{1, 2, \ldots, K\}$，$K$ 是规划时域长度。

**规划时域选择规则**：
$$K > \frac{v_{max}}{a_{max} h}$$

确保机器人可以在时域内将速度从最大值降为零。

---

## 1.2 MBVC-WB：改进的缓冲Voronoi单元

### 1.2.1 预定轨迹定义

**定义（预定轨迹 PT）**：机器人 $i$ 在时刻 $t$ 的预定轨迹定义为：
$$\bar{P}_i(t) = [\bar{p}_i^1(t), \bar{p}_i^2(t), \ldots, \bar{p}_i^K(t)]$$

其中：
- $\bar{p}_i^k(t) = p_i^{k+1}(t-h)$，对于 $k \in \tilde{\mathcal{K}} = \{1, \ldots, K-1\}$
- $\bar{p}_i^K(t) = p_i^K(t-h)$，终端位置

### 1.2.2 MBVC-WB定义

对于任意机器人对 $(i, j)$ 且 $j \neq i$，定义MBVC-WB：

$$V_{ij}^k = \left\{ p \in \mathbb{R}^d \middle| \frac{(p - \bar{p}_i^k + \bar{p}_j^k)^T \bar{p}_{ij}^k}{\|\bar{p}_{ij}^k\|} \leq r_{ij}^k \right\}$$

其中：
- $\bar{p}_{ij}^k = \bar{p}_i^k - \bar{p}_j^k$
- 缓冲宽度：
$$r_{ij}^k = \begin{cases} \frac{\bar{r}_{min}}{2}, & k \in \tilde{\mathcal{K}} \\ \frac{\bar{r}_{min}}{2} + w_{ij}, & k = K \end{cases}$$
- 扩展缓冲宽度：
$$\bar{r}_{min} = \sqrt{r_{min}^2 + h^2 v_{max}^2}$$
- $w_{ij} \in [0, \epsilon]$ 是警告带距离变量，$\epsilon$ 是警告带最大宽度

### 1.2.3 线性约束转换

**非终端时刻约束**：
$$a_{ij}^k \cdot p_i^k \leq b_{ij}^k, \quad j \neq i, k \in \tilde{\mathcal{K}}$$

**终端时刻约束**：
$$a_{ij}^K \cdot p_i^K \leq b_{ij}^K + w_{ij}, \quad j \neq i$$

其中系数为：
$$a_{ij}^k = \frac{\bar{p}_{ij}^k}{\|\bar{p}_{ij}^k\|}, \quad b_{ij}^k = a_{ij}^k \cdot \frac{\bar{p}_i^k + \bar{p}_j^k}{2} + \frac{\bar{r}_{min}}{2}$$

### 1.2.4 安全性引理

**引理1**：如果 $p_i^k \in V_{ij}^k$ 对所有 $i, j \in \mathcal{N}$ ($i \neq j$) 和 $k \in \mathcal{K}$ 成立，则：
$$\|p_{ij}^k\| \geq \bar{r}_{min}$$

且假设机器人 $i, j$ 分别以恒定速度从 $p_i^k$ 移动到 $p_i^{k+1}$ 和从 $p_j^k$ 移动到 $p_j^{k+1}$，则规划轨迹 $P_i(t), P_j(t)$ 在整个轨迹上是无碰撞的。

**证明要点**：在时间区间 $[t+(k-1)h, t+kh]$ 内，机器人间距离满足：
$$\|(1-\lambda)p_i^{k-1} + \lambda p_i^k - p_j^{k-1} - \lambda(p_j^k - p_j^{k-1})\| \geq r_{min}$$

其中 $\lambda \in [0, 1]$。

---

## 1.3 完整优化问题

### 1.3.1 目标函数

目标函数由两部分组成：

$$C_i = C_{w_i} + C_{p_i}$$

**警告带惩罚项**：
$$C_{w_i} = \sum_{j \neq i} \gamma_{ij}\left(\frac{\epsilon}{w_{ij}} - \ln w_{ij}\right)$$

其中 $\gamma_{ij} > 0$ 是死锁解析参数。

**重要性质**：
$$\lim_{w_{ij} \to 0^+} C_{w_i} = +\infty, \quad \left.\frac{\partial C_{w_i}}{\partial w_{ij}}\right|_{w_{ij}=\epsilon} = 0$$

**位置惩罚项**：
$$C_{p_i} = \frac{1}{2}Q_K \|p_i^K - p_i^{target}\|^2 + \frac{1}{2}\sum_{k=0}^{K-1} Q_k \|p_i^{k+1} - p_i^k\|^2$$

其中 $Q_k > 0$ ($k \in \mathcal{K}$) 是权重参数，$Q_0 = 0$。

### 1.3.2 终端约束

为确保优化的可行性，引入终端约束：
$$x_i^K \in \mathcal{X}_e, \quad \text{where } \mathcal{X}_e = \{x | x = Ax + Bu, u \in \mathcal{U}\}$$

对于双积分器模型，等价于：
$$v_i^K = 0_d$$

**备注**：一旦强制执行此约束，规划时域 $K$ 可扩展到无穷大，因为当 $k > K$ 时，规划状态 $x_i^k$ 与 $x_i^K$ 相同（若 $u_i^{k-1} = u_e$）。因此称为无限时域MPC (IMPC)。

### 1.3.3 完整优化问题

$$\min_{\{u_i^{k-1}, x_i^k, w_{ij}\}} C_i$$

$$\text{s.t.} \quad a_{ij}^k \cdot p_i^k \leq b_{ij}^k, \quad j \neq i, k \in \tilde{\mathcal{K}}$$
$$\quad\quad a_{ij}^K \cdot p_i^K \leq b_{ij}^K + w_{ij}, \quad j \neq i$$
$$\quad\quad 0 \leq w_{ij} \leq \epsilon$$
$$\quad\quad v_i^K = 0_d$$
$$\quad\quad x_i^k = Ax_i^{k-1} + Bu_i^{k-1}, \quad k \in \mathcal{K}$$
$$\quad\quad \|v_i^k\|_v \leq v_{max}, \quad k \in \mathcal{K}$$
$$\quad\quad \|u_i^{k-1}\|_a \leq a_{max}, \quad k \in \mathcal{K}$$

---

## 1.4 死锁条件的KKT分析

### 1.4.1 死锁定义

**定义（死锁）**：死锁发生在所有机器人相互阻塞并无限期保持静止，但至少有一个机器人未到达目标位置。

### 1.4.2 死锁条件定理

**定理1**：机器人 $i \in \mathcal{N}$ 属于一个死锁当且仅当以下条件成立：

$$Q_K (p_i^{target} - p_i^K) + \sum_{j \in \mathcal{N}^i} \gamma_{ij} \frac{\alpha_{ij}}{w_{ij}} a_{ij}^K = 0$$

其中：
- $\mathcal{N}^i = \{j | w_{ij} < \epsilon\}$ 是与机器人 $i$ 存在"接触"的机器人集合
- $\alpha_{ij} = \frac{\epsilon - w_{ij}}{w_{ij}}$
- $w_{ij} = w_{ji}$ 对 $j \in \mathcal{N}^i$ 成立

**证明**：

构造Lagrange函数：

$$\mathcal{L}_i = C_i + \sum_{k=1}^{K} \mu_i^k (\|u_i^{k-1}\|_a - a_{max}) + \sum_{k=1}^{K} \nu_i^k (\|v_i^k\|_v - v_{max})$$
$$+ \sum_{j \neq i} \lambda_i^{Kj} (b_i^{Kj} + w_{ij} - a_i^{Kj} \cdot p_i^K) + \sum_{j \neq i} \omega_i^j (w_{ij} - \epsilon)$$
$$+ \sum_{k=1}^{K-1} \sum_{j \neq i} \lambda_i^{kj} (b_i^{kj} - a_i^{kj} \cdot p_i^k) + t_i^T v_i^K$$
$$+ \sum_{k=1}^{K} \kappa_i^k \cdot (x_i^k - Ax_i^{k-1} - Bu_i^{k-1})$$

当死锁发生时，所有机器人静止，即 $u_i^{k-1} = 0_d$ 和 $v_i^k = 0_d$。

根据KKT条件的互补松弛条件：$\mu_i^k = 0$ 和 $\nu_i^k = 0$。

根据KKT条件的稳定性条件：

$$\frac{\partial \mathcal{L}_i}{\partial p_i^k} = \frac{\partial C_i}{\partial p_i^k} - \sum_{j \neq i} \lambda_i^{kj} a_i^{kj} + \kappa_i^k = 0$$

$$\frac{\partial \mathcal{L}_i}{\partial v_i^k} = \nu_i^k + \kappa_i^k = 0, \quad k \in \tilde{\mathcal{K}}$$

$$\frac{\partial \mathcal{L}_i}{\partial v_i^K} = \nu_i^K + t_i = 0$$

$$\frac{\partial \mathcal{L}_i}{\partial u_i^{k-1}} = -(\kappa_i^k)^T A^{K-k}B - (\kappa_i^{k+1})^T A^{K-k-1}B - \ldots - (\kappa_i^K)^T B = 0$$

$$\frac{\partial \mathcal{L}_i}{\partial w_{ij}} = \frac{\partial C_i}{\partial w_{ij}} + \omega_i^j + \lambda_i^{Kj} = 0$$

由 $B$ 的结构可得：
$$(\kappa_i^K)^T B = 0 \Rightarrow v_i^K = 0$$

进一步分析可得 $p_i^K = 0$，然后：

$$\lambda_i^{Kj} = -\frac{\partial C_i}{\partial w_{ij}} = -\gamma_{ij} \frac{\alpha_{ij}}{w_{ij}}$$

代入稳定性条件得到死锁条件。

### 1.4.3 力学解释

死锁条件可以理解为**力平衡**：

$$F_i^A + \sum_{j \in \mathcal{N}^i} F_{ij}^R = 0$$

其中：
- **吸引力**：$F_i^A = Q_K (p_i^{target} - p_i^K)$，来自目标的吸引力，方向指向目标
- **排斥力**：$F_{ij}^R = \gamma_{ij} \alpha_{ij} a_{ij}^K$，来自机器人 $j$ 的排斥力

#### 排斥力公式详细说明

$$F_{ij}^R = \gamma_{ij} \alpha_{ij} a_{ij}^K$$

| 符号 | 名称 | 定义/公式 | 物理意义 |
|------|------|----------|----------|
| $F_{ij}^R$ | 排斥力 | - | 机器人 $j$ 对机器人 $i$ 产生的排斥力，方向从 $j$ 指向 $i$ |
| $\gamma_{ij}$ | 死锁解析参数 | $\gamma_{ij} = \gamma_0 e^{(\rho_i \sin \theta_{ij})}$ | 可主动调整的权重系数，用于打破死锁 |
| $\alpha_{ij}$ | 警告带松弛因子 | $\alpha_{ij} = \frac{\epsilon - w_{ij}}{w_{ij}}$ | 反映两机器人接近程度，越接近排斥力越大 |
| $a_{ij}^K$ | 单位方向向量 | $a_{ij}^K = \frac{\bar{p}_i^K - \bar{p}_j^K}{\|\bar{p}_i^K - \bar{p}_j^K\|}$ | 指向从机器人 $j$ 到机器人 $i$ 的方向 |

**各参数详细解释**：

1. **$\gamma_{ij}$（死锁解析参数）**
   - $\gamma_0 > 0$：排斥力系数基数
   - $\rho_i$：死锁强度参数，随死锁持续时间增长
   - $\theta_{ij}$：从机器人 $i$ 到目标方向与到机器人 $j$ 方向的夹角
   - 右手定则机制：
     - 当 $\theta_{ij} > 0$（机器人在左手边）：$e^{(\rho_i \sin \theta_{ij})} > 1$，排斥力增强
     - 当 $\theta_{ij} < 0$（机器人在右手边）：$e^{(\rho_i \sin \theta_{ij})} < 1$，排斥力减弱

2. **$\alpha_{ij}$（警告带松弛因子）**
   - $\epsilon$：警告带最大宽度（设计参数）
   - $w_{ij}$：当前实际警告带宽度（优化变量）
   - 关键性质：
     - 当 $w_{ij} = \epsilon$（无接触）：$\alpha_{ij} = 0$，无排斥力
     - 当 $w_{ij} \to 0$（接近碰撞边界）：$\alpha_{ij} \to \infty$，排斥力趋向无穷大
   - 这确保了碰撞约束始终满足

3. **$a_{ij}^K$（单位方向向量）**
   - $\bar{p}_i^K$：机器人 $i$ 在终端时刻 $K$ 的预定位置
   - $\bar{p}_j^K$：机器人 $j$ 在终端时刻 $K$ 的预定位置
   - 方向：从 $j$ 指向 $i$，即推开的方向

**排斥力的物理直觉**：

排斥力模拟了机器人之间的"弹簧"作用：
- 当两机器人靠近时（$w_{ij}$ 减小），"弹簧"被压缩，排斥力增大
- 排斥力方向始终是推开的方向（从 $j$ 指向 $i$）
- 通过调整 $\gamma_{ij}$，可以不对称地改变排斥力，实现死锁解析

### 1.4.4 与无警告带情况的对比

**备注4**：如果没有警告带（即 $w_{ij} = 0$），目标函数仅包含 $C_{p_i}$，则死锁的必要条件重写为：
$$Q_K (p_i^{target} - p_i^K) + \sum_{j \neq i} \lambda_i^{Kj} a_{ij}^K = 0$$

此时排斥力大小 $\lambda_i^{Kj}$ 被动确定。

引入警告带变量 $w_{ij}$ 并将其纳入目标函数后，排斥力大小满足：
$$\lambda_i^{Kj} = -\gamma_{ij} \frac{\alpha_{ij}}{w_{ij}}$$

包含 $\gamma_{ij}$ 作为参数。因此，通过适当调整 $\gamma_{ij}$，可以主动调整排斥力来否定死锁条件。

---

## 1.5 死锁检测与解析

### 1.5.1 终端重叠检测

**定义（终端重叠 Terminal Overlap）**：机器人 $i$ 的终端重叠发生在以下条件同时满足时：
1. $p_i^K(t) = p_i^K(t-h)$ — 终端位置不变
2. $p_i^K(t) \neq p_i^{target}$ — 未到达目标
3. $p_i^K(t) = p_i^{K-1}(t)$ — 终端位置与前一个规划位置重合
4. $p_i^{K-1}(t) = p_i^{K-2}(t)$ — 连续两个规划位置重合

此条件允许在死锁实际发生之前的 $(K-2)h$ 时间内进行早期检测。

### 1.5.2 右手定则自适应调整

**排斥力系数调整**：
$$\gamma_{ij} = \gamma_0 e^{(\rho_i(t) \sin \theta_{ij})}$$

**死锁强度参数更新**：
$$\rho_i(t) = \begin{cases}
\rho_i(t-h) + \delta, & \text{if } b_i^{TO} = True \\
0, & \text{if } w_{ij} = \epsilon, \forall j \neq i \\
\rho_i(t-h), & \text{otherwise}
\end{cases}$$

其中：
- $\gamma_0 > 0$ 和 $\delta > 0$ 是设计参数
- 初始值 $\rho_i(t_0) = 0$
- $b_i^{TO} = True$ 表示终端重叠发生
- $\theta_{ij}$ 是在xy平面内，从 $p_i^K$ 到 $p_i^{target}$ 的方向与到 $p_j^K$ 的方向之间的夹角

### 1.5.3 右手定则工作机制

- 当 $\theta_{ij} > 0$（机器人在左手边）：$e^{(\rho_i \sin \theta_{ij})} > 1$，排斥力增强，机器人远离机器人 $j$
- 当 $\theta_{ij} < 0$（机器人在右手边）：$e^{(\rho_i \sin \theta_{ij})} < 1$，排斥力减弱，机器人靠近机器人 $j$

一旦 $w_{ij} = \epsilon$ 对所有 $j \neq i$ 成立，则 $p_i^K$ 必须在其他所有机器人的警告带之外，不存在 $j \in \mathcal{N}^i$。此时 $\rho_i$ 恢复到初始值 $0$。

---

## 1.6 无稳定死锁定理

**定理2**：假设以下条件成立：
1. $\|p_i^{target} - p_j^{target}\| > \bar{r}_{min} + 2\epsilon$，对于 $i \neq j$
2. 三个或更多机器人的目标投影到水平面不共线

则在算法1下不存在稳定死锁。

**证明**：

一旦终端重叠条件成立，$\gamma_{ij} = \gamma_0 e^{(\rho_i(t-h) \sin \theta_{ij})}$ 被替换为 $\gamma_{ij} = \gamma_0 e^{(\rho_i(t) \sin \theta_{ij})}$，其中 $\rho_i(t) = \rho_i(t-h) + \delta$。

建立坐标系：以从机器人 $i$ 到其目标位置的方向为x轴，其正交线为y轴。

假设死锁无限持续。根据定理1，机器人 $i$ 在y方向的合力为：
$$F_y^i = \sum_{j \in \mathcal{N}^i} (-\sin \theta_{ij}) \gamma_0 e^{(\rho_i(t) \sin \theta_{ij})} \gamma_{ij} = 0$$

由于死锁持续且 $\theta_{ij}, \gamma_{ij}$ 不变，在 $t-h$ 时刻的平衡条件为：
$$\sum_{j \in \mathcal{N}^i} (-\sin \theta_{ij}) \gamma_0 e^{(\rho_i(t-h) \sin \theta_{ij})} \gamma_{ij} = 0$$

结合两式：
$$F_y^i = \sum_{j \in \mathcal{N}^i} \sin \theta_{ij} \gamma_0 e^{(\rho_i(t-h) \sin \theta_{ij})} \gamma_{ij} (1 - e^{\delta \sin \theta_{ij}})$$

由于 $\sin \theta_{ij} (1 - e^{\delta \sin \theta_{ij}}) \geq 0$ 对任意 $\theta_{ij} \in (-\pi, \pi]$ 成立，所以 $F_y^i \geq 0$。

等号成立当且仅当 $\theta_{ij} = 0$ 或 $\theta_{ij} = \pi$ 对所有 $j \in \mathcal{N}^i$ 成立。

根据条件2，三个或更多机器人的目标不共线，因此只需考虑两个机器人的情况。通过几何分析可证明两个机器人情况下的死锁是不稳定的。

---

## 1.7 递归可行性

**定理3**：优化问题(12)在算法1下是递归可行的。

**证明**：

给定 $t-h$ 时刻的可行解 $u_i^{k-1}(t-h)$ 和 $x_i^k(t-h)$ ($k \in \mathcal{K}$)，构造 $t$ 时刻的可行解：

$$x_i^k(t) = x_i^{k+1}(t-h), \quad u_i^{k-1}(t) = u_i^k(t-h), \quad k \in \mathcal{K}$$

$$w_{ij}(t) = \min\{\epsilon, a_{ij}^K(t) \cdot p_i^K(t-h) - b_{ij}^K(t)\}$$

验证：
1. 作为 $t-h$ 时刻优化结果，$x_i^{k+1}(t-h)$ 和 $u_i^k(t-h)$ 满足动力学约束
2. 由于 $x_i^K(t) = x_i^{K+1}(t-h) = x_i^K(t-h)$ 和 $u_i^{K-1}(t) = u_i^K(t-h) = u_e$，满足终端约束
3. 由 $x_i^K(t) = x_i^K(t-h) = x_i^{K-1}(t)$，速度约束满足
4. 由引理1，$p_{ij}^k \geq \bar{r}_{min}$，满足碰撞避免约束
5. 由 $\|p_i^K(t-h) - p_j^K(t-h)\| > \bar{r}_{min}$，可得 $w_{ij}(t) > 0$

---

## 1.8 局部通信

**定理4**：如果每个机器人 $i \in \mathcal{N}$ 仅与满足以下条件的机器人 $j \in \mathcal{N}$ 通信：
$$\|p_{ij}\| \leq 2v_{max}Kh + \bar{r}_{min} + 2\epsilon$$

则无死锁、无碰撞和递归可行性的保证仍然成立。

---

## 1.9 完整算法

**算法1：IMPC-DR**

```
Input: p_i(t_0), p_i^{target}
1: b_i^{TO} ← False
2: P_i(t_0) ← InitialPT(p_i(t_0))

3: while not all robots at target do
4:   for i ∈ N concurrently do
5:     P_j(t) ← Communicate(P_i(t))
6:     cons_i ← MBVC-WB(P_i(t), P_j(t))
7:     γ_ij ← DeadlockResolve()
8:     P_i(t) ← Optimization(cons_i, γ_ij)
9:     b_i^{TO} ← DeadlockDetection(P_i(t))
10:    P_i(t+h) ← GetPT(P_i(t))
11:    ExecuteTrajectory(P_i(t))
12:    t ← t + h
```

---

# 第二章：障碍物密集环境中的轨迹规划

## 2.1 障碍物环境问题定义

### 2.1.1 障碍物描述

设 $\mathcal{O} \subset \mathbb{R}^d$ 表示障碍物占据的空间。障碍物假设为凸形状。

自由空间 $\mathcal{S}$ 定义为多个半空间的交集：
$$\mathcal{S} = \{p | a_o^T p > b_o, a_o \neq 0, o \in \mathcal{W}\}$$

其中 $\mathcal{W} = \{1, 2, \ldots, M\}$，$\{p | a_o^T p = b_o\}$ ($o \in \mathcal{W}$) 是障碍物边界。

障碍物空间 $\mathcal{O} = \mathbb{R}^d \setminus \mathcal{S}$。

### 2.1.2 障碍物碰撞避免

安全区域 $R_i = \{x + p_i | \|x\|^2 \leq r_a\}$ 不与障碍物空间相交：
$$R_i \cap \mathcal{O} = \emptyset, \quad \forall i \in \mathcal{N}$$

障碍物膨胀后的空间 $\tilde{\mathcal{O}}$ 由 $\mathcal{O}$ 膨胀半径 $r$ 得到。

规划轨迹 $P$ 无碰撞的条件：
$$\text{Conv}(p_i^k, p_i^{k+1}) \cap \tilde{\mathcal{O}} = \emptyset, \quad k \in \tilde{\mathcal{K}}$$

---

## 2.2 安全走廊构建

### 2.2.1 路径规划

使用ABIT (Advanced Batch Informed Trees) 算法为每个机器人 $i$ 生成参考路径：
$$\xi_i = \{p_i^K, \ldots, p_i^{target}\}$$

连接终端规划位置和目标位置。

### 2.2.2 牵引点选择

牵引点定义为：
$$p_i^{tractive} = \xi_i^m$$

其中 $m$ 是满足以下条件的最大索引：
$$\text{Conv}(\xi_i^m, p_i^K) \cap \tilde{\mathcal{O}} = \emptyset$$

### 2.2.3 扩展预定轨迹(EPT)分段

EPT定义为 $\tilde{P}_i = \{p_i^1, \ldots, p_i^{K+1}\}$，其中 $p_i^{K+1} = p_i^{tractive}$。

分段算法：
1. 初始化 $S_i^1 = \{p_i^{K+1}\}$
2. 从 $k = K$ 到 $k = 1$，将 $p_i^k$ 加入当前段 $S_i^1$，直到凸包 $\text{Conv}(S_i^1)$ 与 $\tilde{\mathcal{O}}$ 相交
3. 初始化下一段 $S_i^2 = \{p_i^k\}$，其中 $p_i^k$ 是 $S_i^1$ 的最后一个点
4. 重复直到所有点分配完毕

### 2.2.4 分离超平面计算

对于第 $n$ 个段 $S_i^n$ 和障碍物，求解优化：

$$\max_{a,b,\eta} \eta$$
$$\text{s.t.} \quad a^T p_S \geq \eta + b, \quad \forall p_S \in S_i^n$$
$$\quad\quad a^T p_O \leq b, \quad \forall p_O \in \text{obstacle}$$
$$\quad\quad \|a\| = 1, \eta \geq 0$$

转换为二次规划(QP)：
$$\min_{a,b} \|a\|^2$$
$$\text{s.t.} \quad a^T p_S \geq 1 + b$$
$$\quad\quad a^T p_O \leq b$$

分离超平面由 $\tilde{a} = \frac{a}{\|a\|}$ 和 $\tilde{b} = \frac{b}{\|a\|}$ 确定。

### 2.2.5 障碍物约束

$$a_{i,o}^k \cdot p_i^k \leq b_{i,o}^k, \quad o \in \mathcal{W}, k \in \tilde{\mathcal{K}}$$
$$a_{i,o}^K \cdot p_i^K \leq b_{i,o}^K + w_{oi}, \quad o \in \mathcal{W}$$

其中：
$$a_{i,o}^k = \frac{a_o}{\|a_o\|}, \quad b_{i,o}^k = r + a_{i,o}^k \cdot p_o$$

$w_{oi} \in [0, \omega]$ 是障碍物警告带变量。

---

## 2.3 障碍物环境中的完整优化问题

### 2.3.1 目标函数

$$C_i = C_{p_i} + C_{a_i} + C_{w_i}$$

**位置惩罚项**：
$$C_{p_i} = \frac{1}{2}Q_K \|p_i^K - p_i^{target}\|^2 + \frac{1}{2}\sum_{k=1}^{K-1} Q_k \|p_i^{k+1} - p_i^k\|^2$$

**机器人间警告带惩罚项**：
$$C_{a_i} = \sum_{j \neq i} \gamma_{ij}\left(\frac{\epsilon}{w_{ij}} - \ln w_{ij}\right)$$

**障碍物警告带惩罚项**：
$$C_{w_i} = \sum_{o \in \mathcal{W}} \gamma_{io}\left(\frac{\omega}{w_{oi}} - \ln w_{oi}\right)$$

### 2.3.2 完整优化

$$\min_{\{U_i, X_i, w_{ij}, w_{oi}\}} C_i$$

$$\text{s.t.} \quad a_{ij}^k \cdot p_i^k \leq b_{ij}^k, \quad j \neq i, k \in \tilde{\mathcal{K}}$$
$$\quad\quad a_{ij}^K \cdot p_i^K \leq b_{ij}^K + w_{ij}, \quad j \neq i$$
$$\quad\quad 0 \leq w_{ij} \leq \epsilon$$
$$\quad\quad v_i^K = 0$$
$$\quad\quad x_i^k = Ax_i^{k-1} + Bu_i^{k-1}, \quad k \in \mathcal{K}$$
$$\quad\quad \|v_i^k\| \leq v_{max}, \quad k \in \mathcal{K}$$
$$\quad\quad \|u_i^{k-1}\| \leq a_{max}, \quad k \in \mathcal{K}$$
$$\quad\quad a_{i,o}^k \cdot p_i^k \leq b_{i,o}^k, \quad o \in \mathcal{W}, k \in \tilde{\mathcal{K}}$$
$$\quad\quad a_{i,o}^K \cdot p_i^K \leq b_{i,o}^K + w_{oi}, \quad o \in \mathcal{W}$$
$$\quad\quad 0 \leq w_{oi} \leq \omega$$

---

## 2.4 障碍物环境中的死锁条件

### 2.4.1 扩展死锁条件定理

**定理**：机器人 $i \in \mathcal{N}$ 属于一个死锁当且仅当以下条件成立：

$$Q_K (p_i^{target} - p_i^K) + \sum_{j \in \mathcal{N}^i} \gamma_{ij} \frac{\alpha_{ij}}{w_{ij}} a_{ij}^K + \sum_{o \in \mathcal{W}^i} \gamma_{io} \frac{\alpha_{io}}{w_{oi}} a_{i,o}^K = 0$$

其中：
- $\mathcal{N}^i = \{j | w_{ij} < \epsilon\}$
- $\mathcal{W}^i = \{o | w_{oi} < \omega\}$
- $\alpha_{ij} = \frac{\epsilon - w_{ij}}{w_{ij}}$
- $\alpha_{io} = \frac{\omega - w_{oi}}{w_{oi}}$

### 2.4.2 力学解释

$$F_i^A + \sum_{j \in \mathcal{N}^i} F_{ij}^R + \sum_{o \in \mathcal{W}^i} F_{io}^R = 0$$

其中：
- **吸引力**：$F_i^A = Q_K (p_i^{target} - p_i^K)$
- **机器人排斥力**：$F_{ij}^R = \gamma_{ij} \alpha_{ij} a_{ij}^K$
- **障碍物排斥力**：$F_{io}^R = \gamma_{io} \alpha_{io} a_{i,o}^K$

#### 机器人排斥力 $F_{ij}^R$ 详细说明

与自由空间中的定义相同，见 [1.4.3节](#143-力学解释)。

#### 障碍物排斥力 $F_{io}^R$ 详细说明

$$F_{io}^R = \gamma_{io} \alpha_{io} a_{i,o}^K$$

| 符号 | 名称 | 定义/公式 | 物理意义 |
|------|------|----------|----------|
| $F_{io}^R$ | 障碍物排斥力 | - | 障碍物 $o$ 对机器人 $i$ 产生的排斥力 |
| $\gamma_{io}$ | 障碍物排斥力系数 | $\gamma_{io} = \frac{1}{\bar{d}_{i,o}^K}$ | 与障碍物距离成反比，距离越近排斥力越大 |
| $\alpha_{io}$ | 障碍物警告带松弛因子 | $\alpha_{io} = \frac{\omega - w_{oi}}{w_{oi}}$ | 反映机器人与障碍物接近程度 |
| $a_{i,o}^K$ | 障碍物分离超平面法向量 | 分离超平面的单位法向量 | 指向远离障碍物的方向 |

**各参数详细解释**：

1. **$\gamma_{io}$（障碍物排斥力系数）**
   - $\bar{d}_{i,o}^K$：机器人预定位置到障碍物边界的距离
   - 距离越近，$\gamma_{io}$ 越大，排斥力越强
   - 这体现了"越接近障碍物，越需要更大的力推开"的物理直觉

2. **$\alpha_{io}$（障碍物警告带松弛因子）**
   - $\omega$：障碍物警告带最大宽度
   - $w_{oi}$：当前实际障碍物警告带宽度
   - 当 $w_{oi} \to 0$（接近障碍物边界）：$\alpha_{io} \to \infty$，排斥力趋向无穷大

3. **$a_{i,o}^K$（分离超平面法向量）**
   - 由安全走廊构建过程中的分离超平面确定
   - 方向始终指向自由空间（远离障碍物）

---

## 2.5 动态优先级机制

### 2.5.1 优先级定义

| 优先级 | 描述 | 条件 |
|--------|------|------|
| 优先级 1 ($\pi_i = 1$) | 最低优先级 | 已到达目标 |
| 优先级 2 ($\pi_i = 2$) | 中等优先级（默认） | 正常导航 |
| 优先级 3 ($\pi_i = 3$) | 最高优先级 | $\rho_i \geq \rho_{max}$ 且满足分配条件 |

### 2.5.2 死锁强度更新

$$\rho_i(t) = \begin{cases}
\rho_i(t-h) + \delta, & \text{if } flag_i = True \\
0, & \text{if } flag_i = False \text{ and } \rho_i(t-h) \geq \rho_{max} \\
\rho_i(t-h), & \text{otherwise}
\end{cases}$$

### 2.5.3 排斥力系数计算（考虑优先级）

$$\gamma_{ij} = \begin{cases}
\gamma_0 e^{(\rho_i \sin \theta_{ij})}, & \text{if } \pi_i = \pi_j \\
\gamma_{min}, & \text{if } \pi_i > \pi_j \\
\gamma_{max} e^{(\sin \theta_{ij})}, & \text{if } \pi_i < \pi_j
\end{cases}$$

其中 $\gamma_{max} \geq \gamma_0 \geq \gamma_{min} > 0$。

### 2.5.4 障碍物排斥力系数

**障碍物到预定位置的距离**：
$$\bar{d}_{i,o}^K = \frac{|a_{i,o}^K \cdot \bar{p}_i^K - b_{i,o}^K|}{2}$$

**障碍物排斥力系数**：
$$\gamma_{io} = \frac{1}{\bar{d}_{i,o}^K} = \frac{|a_{i,o}^K \cdot \bar{p}_i^K - b_{i,o}^K|}{2}$$

**排斥力方向**：
$$n_p^a = \frac{\sum_{o \in \mathcal{W}^i} \gamma_{io} \bar{d}_{i,o}^K a_{i,o}^K}{\|\sum_{o \in \mathcal{W}^i} \gamma_{io} \bar{d}_{i,o}^K a_{i,o}^K\|}$$

---

## 2.6 可行性保证

**定理1**：如果所有机器人初始时相互无碰撞且与障碍物无碰撞，则在算法2下保持如此。

**证明**：

在初始时刻 $t_0$，预定轨迹 $P_i(t_0)$ 显然是优化问题(9)的可行解。

根据引理1，如果 $t-h$ 时刻轨迹规划可行，则 $t-h$ 时刻的规划轨迹无障碍物碰撞，可由此推导安全走廊。

给定 $t-h$ 时刻的可行解，构造 $t$ 时刻的可行解 $x_i^k(t) = x_i^{k+1}(t-h)$，$u_i^k(t) = u_i^{k+1}(t-h)$，其中强制 $x_i^K(t) = x_i^K(t-h)$ 和 $u_i^{K-1}(t) = 0_d$。

---

## 2.7 完整算法

**算法2：障碍物环境完整算法**

```
Input: p_i(t_0), p_i^{target}, O
1: P_i(t_0) ← [p_i(t_0), ..., p_i(t_0)]

2: while not all robots at target do
3:   for i ∈ N concurrently do
4:     obtain Data_j via communication
5:     obtain a_{i,o}^k, b_{i,o}^k via safe corridor construction
6:     γ_ij ← DeadlockResolution(Data_j)
7:     obtain P_i(t) from optimization
8:     send P_i(t) to lower-level controller
9:     t ← t + h
```

---

# 第三章：障碍物排斥力的主动设计

## 3.1 障碍物排斥力的被动与主动设计

### 3.1.1 被动设计

障碍物排斥力系数：
$$\gamma_{io} = \frac{1}{\bar{d}_{i,o}^K}$$

排斥力方向由预定位置和障碍物边界决定。

### 3.1.2 主动设计

主动调整障碍物排斥力系数：
$$\gamma_{io} = \tilde{\gamma}_0 e^{w_i(t)} \cdot \frac{\omega - w_{oi}(t-h)}{\omega w_{oi}(t-h)}$$

**死锁强度更新**：
$$w_i(t) = \begin{cases}
w_i(t-h) + \omega_\delta, & \text{if } w_{oi} < \omega \\
0, & \text{if } w_{oi} = \omega \\
w_i(t-h), & \text{otherwise}
\end{cases}$$

其中 $\tilde{\gamma}_0 > 0$ 和 $\omega_\delta > 0$ 是设计参数，$w_i(t_0) = 0$。

---

## 3.2 主动设计中的排斥力分析

### 3.2.1 合力方向确定

当 $w_{oi} < \omega$ 时，障碍物排斥力合力方向：
$$n_p^a = \frac{\sum_{o \in \mathcal{W}^i} \gamma_{io} \bar{d}_{i,o}^K a_{i,o}^K}{\|\sum_{o \in \mathcal{W}^i} \gamma_{io} \bar{d}_{i,o}^K a_{i,o}^K\|}$$

代入 $\gamma_{io}$ 的表达式：
$$n_p^a = \frac{\sum_{o \in \mathcal{W}^i} \tilde{\gamma}_0 e^{w_i(t)} \bar{d}_{i,o}^K(\bar{p}_i^K(t)) \frac{\omega - w_{oi}(t-h)}{\omega w_{oi}(t-h)} a_{i,o}^K}{\|\cdot\|}$$

### 3.2.2 时刻间变化分析

在 $t$ 时刻，$n_p^a$ 在 $x$ 和 $y$ 方向的分量为 $(n_p^a)_x$ 和 $(n_p^a)_y$。

由于 $w_i(t) > w_i(t-h)$ 和 $\bar{d}_{i,o}^K(\bar{p}_i^K(t)) = \bar{d}_{i,o}^K(\bar{p}_i^K(t-h))$，$n_p^a$ 方向在相邻时刻不变。

障碍物排斥力在 $x$ 方向的变化：
$$F_x^i = F_{i,x}^A + \sum_{o \in \mathcal{W}^i} \tilde{\gamma}_0 e^{w_i(t)} \bar{d}_{i,o}^K \frac{\omega - w_{oi}(t)}{\omega w_{oi}(t)} a_{i,o}^K \cdot (n_p^a)_x + \ldots$$

由 $w_i(t) = w_i(t-h) + \omega_\delta$，可得 $x$ 方向力的变化非零，打破平衡。

---

## 3.3 主动设计中的机器人排斥力

### 3.3.1 排斥力系数更新

$$\gamma_{ij} = \gamma_0 e^{(\rho_i(t) \sin \theta_{ij})}$$

**死锁强度更新**：
$$\rho_i(t) = \begin{cases}
\rho_i(t-h) + \delta, & \text{if } b_i^{TO} = True \\
0, & \text{if } w_{ij} = \epsilon, j \neq i \\
\rho_i(t-h), & \text{otherwise}
\end{cases}$$

### 3.3.2 合力方向

当 $w_{oi} < \omega$ 时，定义合力方向：
$$n_p^a = \sum_{o \in \mathcal{W}^i} \gamma_{io} \bar{d}_{i,o}^K a_{i,o}^K$$

当 $w_{oi} = \omega$ 时：
$$n_p^a = p_i^{target} - \bar{p}_i^K$$

### 3.3.3 角度计算

$$\theta_{ij} = \angle(n_p^a, p_j^K - p_i^K)$$

其中角度范围 $\theta_{ij} \in (-\pi, \pi]$。

---

## 3.4 完整的主动设计算法

### 3.4.1 死锁解析算法

**算法：主动设计的$\gamma_{ij}$计算**

```
Input: P̄_i, p_i^{target}, b_i^{OP}, b_i^{OHP}, w_{oi}, ρ_i(t), P̄_j, b_j^{OP}, b_j^{OHP}

1: b_i^{TO} ← 死锁检测
2: if b_i^{TO} = True & w_{oi} < ω, o ∈ W then
3:     计算 θ_ij 根据 (4.12)
4:     if b_i^{OP} = False & b_j^{OP} = True & sin θ_ij < 0 then
5:         γ_ij ← γ_0 e^{(-ρ_max sin θ_ij)}
6:     else if b_i^{OP} = True & b_j^{OP} = True & b_j^{OHP} = True & sin θ_ij = 0 then
7:         γ_ij ← ρ_max e^{(sin θ_ij)}
8:     else
9:         γ_ij ← γ_0 e^{(ρ_i(t) sin θ_ij)}
10:    end
11: else if b_i^{TO} = True & w_{oi} = ω, o ∈ W then
12:    计算 θ_ij 根据 (4.12)
13:    if b_i^{OP} = False & b_j^{OP} = True & sin θ_ij < 0 then
14:        γ_ij ← γ_0 e^{(-ρ_max sin θ_ij)}
15:    else
16:        γ_ij ← γ_0 e^{(ρ_i(t) sin θ_ij)}
17:    end
18: else
19:    γ_ij ← γ_0 e^{(ρ_i(t) sin θ_ij)}
20: end
21: return γ_ij
```

### 3.4.2 主算法

**算法：主动设计完整算法**

```
Input: p_i(t_0), p_i^{target}, O
1: P̄_i(t_0) ← [p_i(t_0), ..., p_i(t_0)]

2: while 未完成 do
3:   for i ∈ N 并行执行 do
4:     获取 P̄_j (j ≠ i)
5:     计算 γ_{io}
6:     构建 MBVC-WB
7:     计算 γ_{ij} (使用算法2)
8:     求解优化问题 (3.17)
9:     更新 b_i^{OP} 和 b_i^{OHP}
10: end
11: t ← t + h
12 end
```

---

## 3.5 无稳定死锁证明

### 3.5.1 定理条件

**定理**：假设以下条件成立：
1. 对所有 $i \neq j$，$\|p_i^{target} - p_j^{target}\| > \bar{d}_{min} + 2\epsilon$
2. 对所有 $i \in \mathcal{N}$ 和 $o \in \mathcal{W}$，$\frac{|a_{i,o}^k \cdot p_i^{target} - b_{i,o}^k|}{\|a_{i,o}^k\|} > \omega$
3. 三个或更多机器人的目标投影到水平面不共线

则在主动设计算法下不存在稳定死锁。

### 3.5.2 证明概要

设机器人 $i$ 处于死锁状态。在 $t-h$ 和 $t$ 时刻，死锁条件成立：
$$F_i^A + \sum_{o \in \mathcal{W}^i} F_{io}^R + \sum_{j \in \mathcal{Q}_i} F_{ij}^R + \sum_{j \in \mathcal{N}^i \setminus \mathcal{Q}_i} F_{ij}^R = 0$$

其中 $\mathcal{Q}_i \subset \mathcal{N}^i$ 是具有优先级差异的机器人集合。

在 $y$ 方向：
$$F_y^i = F_{i,y}^A + \sum_{j \in \mathcal{Q}_i} (-\sin \theta_{ij}) F_{ij}^R + \sum_{j \in \mathcal{N}^i \setminus \mathcal{Q}_i} (-\sin \theta_{ij}) \gamma_0 e^{\rho_i(t) \sin \theta_{ij}} \gamma_{ij} = 0$$

由 $\rho_i(t) = \rho_i(t-h) + \delta$，推导：
$$F_y^i = \sum_{j \in \mathcal{N}^i \setminus \mathcal{Q}_i} \sin \theta_{ij} \gamma_0 e^{\rho_i(t-h) \sin \theta_{ij}} \gamma_{ij} (1 - e^{\delta \sin \theta_{ij}}) \geq 0$$

等号成立当且仅当所有 $\theta_{ij} \in \{0, \pi\}$。

通过几何分析证明等号成立时死锁不满足定理条件或为不稳定死锁。

---

# 第四章：统一方法总结

## 4.1 核心方法框架

我们的方法统称为 **IMPC-DR (Infinite-horizon Model Predictive Control with Deadlock Resolution)**，包含以下核心组件：

### 4.1.1 MBVC-WB空间划分

| 特性 | 传统BVC | MBVC-WB |
|------|---------|---------|
| 空间划分依据 | 仅当前时刻位置 | 所有未来规划位置 |
| 缓冲宽度 | 固定值 $\frac{r_{min}}{2}$ | 速度依赖 $\frac{\bar{r}_{min}}{2}$ |
| 终端约束 | 无警告带 | 添加警告带 $w_{ij}$ / $w_{oi}$ |
| 空间利用率 | 较低 | 较高 |

### 4.1.2 死锁条件统一形式

自由空间：
$$F_i^A + \sum_{j \in \mathcal{N}^i} F_{ij}^R = 0$$

障碍物空间：
$$F_i^A + \sum_{j \in \mathcal{N}^i} F_{ij}^R + \sum_{o \in \mathcal{W}^i} F_{io}^R = 0$$

### 4.1.3 死锁解析策略对比

| 策略 | 适用场景 | 核心机制 |
|------|----------|----------|
| 右手定则 | 自由空间 | 自适应调整 $\gamma_{ij}$ |
| 动态优先级 | 障碍物空间 | 优先级分配 + 右手定则 |
| 主动设计 | 障碍物空间 | 同时调整 $\gamma_{ij}$ 和 $\gamma_{io}$ |

## 4.2 理论保证总结

| 定理 | 内容 |
|------|------|
| 死锁条件定理 | 死锁等价于力平衡条件 |
| 无稳定死锁定理 | 在温和条件下不存在稳定死锁 |
| 递归可行性定理 | 优化问题在所有时刻可行 |
| 局部通信定理 | 仅需局部通信即可保证所有性质 |

## 4.3 算法复杂度分析

| 组件 | 复杂度 |
|------|--------|
| MBVC-WB约束构造 | $O(KN)$ |
| 安全走廊构建 | $O(K \cdot n_{proximal} \cdot (d+n+1))$ |
| 最终优化 | QCQP: $(Kd + N - 1)$ 变量, $(N + n_{proximal} + 1)K + 2N - 1$ 约束 |

## 4.4 参数设置指南

| 参数 | 典型值 | 说明 |
|------|--------|------|
| $v_{max}$ | 1.0 m/s | 最大速度 |
| $a_{max}$ | 1.5 m/s² | 最大加速度 |
| $h$ | 0.2 s | 采样时间 |
| $K$ | 10-12 | 规划时域长度 |
| $r_{min}$ / $d_{min}$ | 0.3 m | 最小安全距离 |
| $\epsilon$ | 0.1 m | 机器人警告带最大宽度 |
| $\omega$ | 0.25 m | 障碍物警告带最大宽度 |
| $Q_K$ | 30.0 | 位置惩罚权重 |
| $\gamma_0$ | 2.0 | 排斥力系数基数 |
| $\delta$ | 0.2 | 死锁强度增量 |
| $\rho_{max}$ | 0.3 | 死锁强度上限 |

---

**代码仓库**：
- 自由空间版本：https://github.com/PKU-MACDLab/IMPC-DR
- 障碍物空间版本：https://github.com/PKU-MACDLab/IMPCOB
