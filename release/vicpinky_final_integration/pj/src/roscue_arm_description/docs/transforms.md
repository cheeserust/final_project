# Transformation Matrices - roscue_arm

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | first_link |
| $L_{2}$ | second_link |
| $L_{3}$ | third_link |
| $L_{4}$ | fourth_link |
| $L_{5}$ | gripper_base_link |
| $L_{6}$ | hand_finger_link_1_1_1 |
| $L_{7}$ | hand_finger_link_2_1_1 |
| $L_{8}$ | hand_finger_link_3_1_1 |
| $L_{9}$ | finger1_link_1_1 |
| $L_{10}$ | finger2_link_1_1 |
| $L_{11}$ | finger3_link_1_1 |
| $L_{12}$ | finger1_link_2_1 |
| $L_{13}$ | finger2_link_2_1 |
| $L_{14}$ | finger3_link_2_1 |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | base_joint | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | arm_joint_1 | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | arm_joint_2 | continuous (rad) | $L_{2}$ | $L_{3}$ |
| $q_{4}$ | arm_joint_3 | continuous (rad) | $L_{3}$ | $L_{4}$ |
| $q_{5}$ | arm_joint_4 | continuous (rad) | $L_{4}$ | $L_{5}$ |
| $q_{6}$ | finger_1_base_joint | continuous (rad) | $L_{5}$ | $L_{6}$ |
| $q_{7}$ | finger_2_base_joint | continuous (rad) | $L_{5}$ | $L_{7}$ |
| $q_{8}$ | finger_3_base_joint | continuous (rad) | $L_{5}$ | $L_{8}$ |
| $q_{9}$ | finger_1_middle_joint | continuous (rad) | $L_{6}$ | $L_{9}$ |
| $q_{10}$ | finger_2_middle_joint | continuous (rad) | $L_{7}$ | $L_{10}$ |
| $q_{11}$ | finger_3_middle_joint | continuous (rad) | $L_{8}$ | $L_{11}$ |
| $q_{12}$ | finger_1_tip_joint | continuous (rad) | $L_{9}$ | $L_{12}$ |
| $q_{13}$ | finger_2_tip_joint | continuous (rad) | $L_{10}$ | $L_{13}$ |
| $q_{14}$ | finger_3_tip_joint | continuous (rad) | $L_{11}$ | $L_{14}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [continuous] base_joint (q1)
      L1: first_link
        +-- [continuous] arm_joint_1 (q2)
            L2: second_link
              +-- [continuous] arm_joint_2 (q3)
                  L3: third_link
                    +-- [continuous] arm_joint_3 (q4)
                        L4: fourth_link
                          +-- [continuous] arm_joint_4 (q5)
                              L5: gripper_base_link
                                |-- [continuous] finger_1_base_joint (q6)
                                |   L6: hand_finger_link_1_1_1
                                |     +-- [continuous] finger_1_middle_joint (q9)
                                |         L9: finger1_link_1_1
                                |           +-- [continuous] finger_1_tip_joint (q12)
                                |               L12: finger1_link_2_1
                                |-- [continuous] finger_2_base_joint (q7)
                                |   L7: hand_finger_link_2_1_1
                                |     +-- [continuous] finger_2_middle_joint (q10)
                                |         L10: finger2_link_1_1
                                |           +-- [continuous] finger_2_tip_joint (q13)
                                |               L13: finger2_link_2_1
                                +-- [continuous] finger_3_base_joint (q8)
                                    L8: hand_finger_link_3_1_1
                                      +-- [continuous] finger_3_middle_joint (q11)
                                          L11: finger3_link_1_1
                                            +-- [continuous] finger_3_tip_joint (q14)
                                                L14: finger3_link_2_1
```

## Transforms

## base_joint

$L_{0}$ **base_link** -> $L_{1}$ **first_link** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0, 0, 0.027) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 0, -1)

### Local Transform

$$
T^{0}_{1}(q_{1}) = \begin{bmatrix}
c_{1} & s_{1} & 0 & 0 \\
-s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0.027 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## arm_joint_1

$L_{1}$ **first_link** -> $L_{2}$ **second_link** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (-0.065, 0, 0.063) m
- **origin rpy**: (0, -1.570796, 0) rad
- **axis**: (0, 0, -1)

### Local Transform

$T^{1}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & -1 & -0.065 \\
0 & 1 & 0 & 0 \\
1 & 0 & 0 & 0.063 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & s_{2} & 0 & 0 \\
-s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## arm_joint_2

$L_{2}$ **second_link** -> $L_{3}$ **third_link** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0.3405, 0, 0.0585) m
- **origin rpy**: (-1.570796, 0, -1.570796) rad
- **axis**: (0, 1, 0)

### Local Transform

$T^{2}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & 0 & 1 & 0.3405 \\
-1 & 0 & 0 & 0 \\
0 & -1 & 0 & 0.0585 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & 0 & s_{3} & 0 \\
0 & 1 & 0 & 0 \\
-s_{3} & 0 & c_{3} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## arm_joint_3

$L_{3}$ **third_link** -> $L_{4}$ **fourth_link** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (-0.0005, 0.0195, 0.18) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, 1, 0)

### Local Transform

$$
T^{3}_{4}(q_{4}) = \begin{bmatrix}
c_{4} & 0 & s_{4} & -0.0005 \\
0 & 1 & 0 & 0.0195 \\
-s_{4} & 0 & c_{4} & 0.18 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## arm_joint_4

$L_{4}$ **fourth_link** -> $L_{5}$ **gripper_base_link** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0, 0.0025, 0.09) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{4}_{5}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & 0 \\
1 & 0 & 0 & 0.0025 \\
0 & 0 & 1 & 0.09 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_1_base_joint

$L_{5}$ **gripper_base_link** -> $L_{6}$ **hand_finger_link_1_1_1** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (-0.000308, 0.047572, 0.0114) m
- **origin rpy**: (0, 0, 1.570796) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{5}_{6}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0 & -1 & 0 & -0.000308 \\
1 & 0 & 0 & 0.047572 \\
0 & 0 & 1 & 0.0114 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & -s_{6} & 0 & 0 \\
s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_2_base_joint

$L_{5}$ **gripper_base_link** -> $L_{7}$ **hand_finger_link_2_1_1** (continuous)
  Variable: $q_{7}$

- **origin xyz**: (0.040983, -0.024158, 0.0152) m
- **origin rpy**: (0, 0, -0.523599) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{5}_{7}(q_{7}) = T_{fixed} \cdot R_{axis}(q_{7})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.866025 & 0.5 & 0 & 0.040983 \\
-0.5 & 0.866025 & 0 & -0.024158 \\
0 & 0 & 1 & 0.0152 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{7}) = \begin{bmatrix}
c_{7} & -s_{7} & 0 & 0 \\
s_{7} & c_{7} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_3_base_joint

$L_{5}$ **gripper_base_link** -> $L_{8}$ **hand_finger_link_3_1_1** (continuous)
  Variable: $q_{8}$

- **origin xyz**: (-0.040913, -0.024275, 0.0152) m
- **origin rpy**: (0, 0, -2.617994) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{5}_{8}(q_{8}) = T_{fixed} \cdot R_{axis}(q_{8})$ where:

$$
T_{fixed} = \begin{bmatrix}
-0.866025 & 0.5 & 0 & -0.040913 \\
-0.5 & -0.866025 & 0 & -0.024275 \\
0 & 0 & 1 & 0.0152 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{8}) = \begin{bmatrix}
c_{8} & -s_{8} & 0 & 0 \\
s_{8} & c_{8} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_1_middle_joint

$L_{6}$ **hand_finger_link_1_1_1** -> $L_{9}$ **finger1_link_1_1** (continuous)
  Variable: $q_{9}$

- **origin xyz**: (0.03505, 0.02115, 0.0608) m
- **origin rpy**: (0, 0, -3.141593) rad
- **axis**: (0, -1, 0)

### Local Transform

$T^{6}_{9}(q_{9}) = T_{fixed} \cdot R_{axis}(q_{9})$ where:

$$
T_{fixed} = \begin{bmatrix}
-1 & 0 & 0 & 0.03505 \\
0 & -1 & 0 & 0.02115 \\
0 & 0 & 1 & 0.0608 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{9}) = \begin{bmatrix}
c_{9} & 0 & -s_{9} & 0 \\
0 & 1 & 0 & 0 \\
s_{9} & 0 & c_{9} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_2_middle_joint

$L_{7}$ **hand_finger_link_2_1_1** -> $L_{10}$ **finger2_link_1_1** (continuous)
  Variable: $q_{10}$

- **origin xyz**: (0.03505, 0.01415, 0.057) m
- **origin rpy**: (0, 0, -3.141593) rad
- **axis**: (0, -1, 0)

### Local Transform

$T^{7}_{10}(q_{10}) = T_{fixed} \cdot R_{axis}(q_{10})$ where:

$$
T_{fixed} = \begin{bmatrix}
-1 & 0 & 0 & 0.03505 \\
0 & -1 & 0 & 0.01415 \\
0 & 0 & 1 & 0.057 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{10}) = \begin{bmatrix}
c_{10} & 0 & -s_{10} & 0 \\
0 & 1 & 0 & 0 \\
s_{10} & 0 & c_{10} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_3_middle_joint

$L_{8}$ **hand_finger_link_3_1_1** -> $L_{11}$ **finger3_link_1_1** (continuous)
  Variable: $q_{11}$

- **origin xyz**: (0.03505, 0.02115, 0.057) m
- **origin rpy**: (0, 0, 3.141593) rad
- **axis**: (0, -1, 0)

### Local Transform

$T^{8}_{11}(q_{11}) = T_{fixed} \cdot R_{axis}(q_{11})$ where:

$$
T_{fixed} = \begin{bmatrix}
-1 & 0 & 0 & 0.03505 \\
0 & -1 & 0 & 0.02115 \\
0 & 0 & 1 & 0.057 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{11}) = \begin{bmatrix}
c_{11} & 0 & -s_{11} & 0 \\
0 & 1 & 0 & 0 \\
s_{11} & 0 & c_{11} & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_1_tip_joint

$L_{9}$ **finger1_link_1_1** -> $L_{12}$ **finger1_link_2_1** (continuous)
  Variable: $q_{12}$

- **origin xyz**: (-0.063, -0.001525, -0.0003) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, -1, 0)

### Local Transform

$$
T^{9}_{12}(q_{12}) = \begin{bmatrix}
c_{12} & 0 & -s_{12} & -0.063 \\
0 & 1 & 0 & -0.001525 \\
s_{12} & 0 & c_{12} & -0.0003 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_2_tip_joint

$L_{10}$ **finger2_link_1_1** -> $L_{13}$ **finger2_link_2_1** (continuous)
  Variable: $q_{13}$

- **origin xyz**: (-0.063, -0.001525, -0.0003) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, -1, 0)

### Local Transform

$$
T^{10}_{13}(q_{13}) = \begin{bmatrix}
c_{13} & 0 & -s_{13} & -0.063 \\
0 & 1 & 0 & -0.001525 \\
s_{13} & 0 & c_{13} & -0.0003 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## finger_3_tip_joint

$L_{11}$ **finger3_link_1_1** -> $L_{14}$ **finger3_link_2_1** (continuous)
  Variable: $q_{14}$

- **origin xyz**: (-0.063, -0.001525, -0.0003) m
- **origin rpy**: (0, 0, 0) rad
- **axis**: (0, -1, 0)

### Local Transform

$$
T^{11}_{14}(q_{14}) = \begin{bmatrix}
c_{14} & 0 & -s_{14} & -0.063 \\
0 & 1 & 0 & -0.001525 \\
s_{14} & 0 & c_{14} & -0.0003 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{second_link})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{third_link})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4})\quad (L_0 \to L_{4}: \text{fourth_link})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5})\quad (L_0 \to L_{5}: \text{gripper_base_link})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6})\quad (L_0 \to L_{6}: \text{hand_finger_link_1_1_1})$$

$$T^{0}_{7} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{7}(q_{7})\quad (L_0 \to L_{7}: \text{hand_finger_link_2_1_1})$$

$$T^{0}_{8} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{8}(q_{8})\quad (L_0 \to L_{8}: \text{hand_finger_link_3_1_1})$$

$$T^{0}_{9} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6}) \cdot T^{6}_{9}(q_{9})\quad (L_0 \to L_{9}: \text{finger1_link_1_1})$$

$$T^{0}_{10} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{7}(q_{7}) \cdot T^{7}_{10}(q_{10})\quad (L_0 \to L_{10}: \text{finger2_link_1_1})$$

$$T^{0}_{11} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{8}(q_{8}) \cdot T^{8}_{11}(q_{11})\quad (L_0 \to L_{11}: \text{finger3_link_1_1})$$

$$T^{0}_{12} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6}) \cdot T^{6}_{9}(q_{9}) \cdot T^{9}_{12}(q_{12})\quad (L_0 \to L_{12}: \text{finger1_link_2_1})$$

$$T^{0}_{13} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{7}(q_{7}) \cdot T^{7}_{10}(q_{10}) \cdot T^{10}_{13}(q_{13})\quad (L_0 \to L_{13}: \text{finger2_link_2_1})$$

$$T^{0}_{14} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{8}(q_{8}) \cdot T^{8}_{11}(q_{11}) \cdot T^{11}_{14}(q_{14})\quad (L_0 \to L_{14}: \text{finger3_link_2_1})$$

