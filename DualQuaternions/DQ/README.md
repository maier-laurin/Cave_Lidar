# DQ.jl
DQ.jl is a lightweight, Julia package for Dual Quaternion algebra. It is specifically designed for rigid body kinematics, offering type-stable transformations and optimized point-in-space processing.

## Key Features
* Hierarchical Type System: Separate types for standard, Unit, Dual, and Unit Dual Quaternions to ensure mathematical invariants are preserved.
* Automatic Promotion: Seamlessly mix precisions (e.g., Float32 and Float64) and types (e.g., Point and UnitDualQuaternion).
* Performance Optimized: Point transformations utilize specialized sandwich products to minimize floating-point operations.
* Type Stability: Multiplying two UnitDualQuaternion transforms guaranteed to return a UnitDualQuaternion.

## Mathematical Overview
A Dual Quaternion is represented as: $$\hat{q} = r + \epsilon d$$ where $r$ is a rotation quaternion and $d$ is the dual part containing translation information. For a Unit Dual Quaternion (a Rigid Body Transform): $r$ is a UnitQuaternion. $d = \frac{1}{2} t r$, where $t$ is the translation vector. Points in 3D space $(x, y, z)$ are embedded in dual space as:$$\hat{p} = 1 + \epsilon (xi + yj + zk)$$

## Usage Examples

1. Creating Rotations and Translations

~~~
using DQ
# 90-degree rotation around the Z-axis
half_angle = π/4
r = UnitQuaternion(cos(half_angle), 0.0, 0.0, sin(half_angle))
~~~
~~~
# A transform that rotates by 'r' and translates by (5, 0, 0)
T = UnitDualQuaternion(r, 5.0, 0.0, 0.0)
~~~

2. Transforming Points

The package provides an optimized transform function that is faster than the general $Q p Q^*$ product.
~~~~
p = Point(1.0, 0.0, 0.0)

# Apply the transformation
p_new = transform(T, p)

println(p_new) # Resulting point in space
~~~~

3. Composing Transformations

Transformations can be composed using the multiplication operator *. The result of multiplying two transforms is itself a UnitDualQuaternion.
~~~
T1 = UnitDualQuaternion(r_id, 10.0, 0.0, 0.0) # Move 10 in X
T2 = UnitDualQuaternion(r_90z, 0.0, 0.0, 0.0) # Rotate 90 in Z

# Combine: Rotate THEN translate
T_total = T1 * T2
~~~

4. Smooth Interpolation (ScLERP)

When you need to transition smoothly between two poses. Unlike standard linear interpolation, ScLERP follows the shortest screw motion in 3D space.
~~~
T_start = UnitDualQuaternion(UnitQuaternion(1, 0, 0, 0), 0, 0, 0)
s2 = 1/sqrt(2)
T_end = UnitDualQuaternion(UnitQuaternion(s2, 0, 0, s2), 10.0, 0.0, 0.0)

# Interpolate at t = 0.5
T_mid = sclerp(T_start, T_end, 0.5)

# Verify the result
p = Point(0, 0, 0)
p_mid = transform(T_mid, p)
~~~

# API Reference
## Types

| Type | Description |
| :-- | :-- | 
|Quaternion{T} | General 4-component quaternion.|
|UnitQuaternion{T} | Normalized quaternion (Rotation).|
|DualQuaternion{T} | General dual quaternion $r + \epsilon d$.|
|UnitDualQuaternion{T} | Rigid body transform (unit real part).|
|Point{T} | 3D coordinate vector $(x, y, z)$.|

## Functions
* *transform(dq, point):* Applies the dual quaternion transform to a point.
* *sclerp(dq1, dq2, t):* finds the interpolated transformation between two transforms.