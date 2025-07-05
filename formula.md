

## 1. 像素坐标 → 相机坐标系射线

设相机内参  

$$
K=\begin{bmatrix}
f_x & 0   & c_x\\
0   & f_y & c_y\\
0   & 0   & 1
\end{bmatrix},
$$

像素点 $(u,v)$ 归一化得  

$$
\tilde{\mathbf d}=
\begin{bmatrix}
(u-c_x)/f_x\\
(v-c_y)/f_y\\
1
\end{bmatrix},
\qquad
\mathbf d_{\text{cam}}
=\frac{\tilde{\mathbf d}}{\|\tilde{\mathbf d}\|}.
$$

---

## 2. 相机坐标系 → ArUco（世界）坐标系

`estimatePoseSingleMarkers` 返回姿态  

$$
\mathbf p_{\text{cam}}=R\,\mathbf p_{\text{aruco}}+\mathbf t.
$$

- 相机光心在世界系中的坐标  

$$
\boxed{\mathbf c_{\text{aruco}}=-R^{\mathsf T}\mathbf t}.
$$

- 射线方向转到世界系  

$$
\boxed{\mathbf d_{\text{aruco}}=R^{\mathsf T}\mathbf d_{\text{cam}} }.
$$

- 射线参数方程  

$$
\boxed{\mathbf p(s)=\mathbf c_{\text{aruco}}+s\,\mathbf d_{\text{aruco}},\;s\ge0}.
$$

---

## 3. 射线与已知平面求交

平面 $\Pi$ 由法向量 $\mathbf n$ 与平面点 $\mathbf p_0$ 描述：  

$$
\mathbf n\cdot(\mathbf p-\mathbf p_0)=0.
$$

将射线代入，求参数  

$$
\boxed{s=\dfrac{\mathbf n\cdot(\mathbf p_0-\mathbf c_{\text{aruco}})}
               {\mathbf n\cdot\mathbf d_{\text{aruco}}}}.
$$

若分母近 0 ⇒ 平行；若 $s<0$ ⇒ 交点在相机后；两者均无效。  
有效时交点为  

$$
\boxed{\mathbf p_\ast=\mathbf c_{\text{aruco}}+s\,\mathbf d_{\text{aruco}} }.
$$

---

## 4. 派生高度等量

若取 ArUco 的 $X$ 轴为竖直方向，已知标记原点到地面高度 $H_{\text{marker}}$，则  

$$
\boxed{H = H_{\text{marker}} + p_\ast^{(x)}}.
$$

如需平移到新原点 $\mathbf o$：  

$$
\boxed{\mathbf p_{\text{new}}=\mathbf p_\ast-\mathbf o }.
$$

---


---

## 6. 小结

1. **像素反投影**：$(u,v)$ → 射线 $\mathbf d_{\text{cam}}$。  
2. **坐标系变换**：利用 ArUco 姿态，将射线与光心转到世界系。  
3. **几何约束**：假设球在已知平面，射线与平面求交得 $\mathbf p_\ast$。  
4. **派生量计算**：得到绝对高度 $H$ 及其他所需物理量。

单目亦可实现 3D 定位，关键在于利用已知平面与标记姿态提供的深度约束。