# LaTeX 渲染测试

## 正确的格式

这是行内公式：$E = mc^2$

这是块级公式：

$$
\mathbf{M}(q) \ddot{\mathbf{q}} + \mathbf{C}(q, \dot{\mathbf{q}}) \dot{\mathbf{q}} + \mathbf{g}(q) = \mathbf{S} \boldsymbol{\tau} + \mathbf{J}^T \mathbf{f}_c
$$

带说明的公式：

$$
\min_{\mathbf{u}} \sum_{k=0}^{N-1} \left[ (\mathbf{x}_k - \mathbf{x}_{ref,k})^T \mathbf{Q} (\mathbf{x}_k - \mathbf{x}_{ref,k}) + (\mathbf{u}_k - \mathbf{u}_{ref,k})^T \mathbf{R} (\mathbf{u}_k - \mathbf{u}_{ref,k}) \right]
$$

其中：
- $\mathbf{x}_k$：状态变量
- $\mathbf{u}_k$：控制输入
- $\mathbf{Q}$, $\mathbf{R}$：权重矩阵

## 错误的格式

### 错误1：$$ 后没有空行

$$
x = y
$$

### 错误2：公式中有前导空格

$$
\mathbf{M}(q): \text{inertia matrix}
$$

### 错误3：使用代码块
```

$$
x = y
$$

```
