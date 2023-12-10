<script type="text/javascript" id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

Welcome to this guide for constructing and operating a kinesthetic haptic controller that harnesses the dynamic properties of a flywheel with dual axes of rotation. This page serves as a one-stop resource, offering all the essential files, codes, and  instructions necessary to create your own gyroscope-based haptic device.

For a head start, we've provided a fully-realized 3D model, ready for 3D printing, accessible via the Fusion website. You can explore the model in detail through the embedded frame below or download it from the dropdown menu:

<div style="text-align:center;">
<iframe width="800" height="450" src="https://myhub.autodesk360.com/ue2901c7f/g/shares/SHd38bfQT1fb47330c99796b3271601d0da5" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

<p>Check out our project on GitHub: <a href="https://github.com/telenaco/Teensy-Gyro" target="_blank">Teensy-Gyro Repository</a></p>

In the following sections, we'll delve deep into the theoretical underpinnings of the device, ensuring you have a robust understanding of the principles that drive its functionality.

# Understanding the Theory

If you are new to gyroscopes, this video will help you understand the output torques from a gyroscope.

<p align="center">
<iframe width="800" height="450" src="https://www.youtube.com/embed/jQEKhIovKA0" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</p>

For a more in-depth mathematical approach, check out this video series:

<p align="center">
<iframe width="800" height="450" src="https://www.youtube.com/embed/bYF0PGsF92k" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</p>

Further details that are covered in this tutorial are explained more comprehensively in the class notes for the [aeronautics-and-astronautics MIT course](https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-07-dynamics-fall-2009/lecture-notes/), particularly from lectures 26 onwards.

## HapticWhirl Modelling

The HapticWhirl device operates through the actuation of a flywheel and a gimbal, each contributing to the generation of torque.

### Momentum Wheel

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled.png" width="560" alt="Momentum Wheel">
    <br>
    <em>Figure - Momentum Wheel</em>
</p>

The flywheel's momentum, which is the force that keeps it rotating, always acts perpendicular to its rotation plane. The torque, or the rotational force exerted by the flywheel, can be calculated with this equation:

$$
\tag{1}\overrightarrow{\tau} = \frac{d\overrightarrow{L}}{dt} = \mathbf{I} \frac{d}{dt}\overrightarrow{\omega}^{\text{disk}} = \mathbf{I} \dot{\overrightarrow{\omega}}^{\text{disk}}
$$

This formula shows that the torque is a result of multiplying the flywheel's moment of inertia (resistance to changes in its rotation) by the rate at which its angular velocity changes.

### Steered Momentum Wheel

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%201.png" width="560" alt="3D model of a steered momentum wheel with labeled parts including the disk and gimbal, showcasing the orientation of applied forces and rotation vectors.">
    <br>
    <em>Figure - Steered Momentum Wheel</em>
</p>

The steered momentum wheel concept refers to the additional momentum generated when both the disk and the gimbal are moving. This is described by the equation:

$$
\tag{2}\overrightarrow{\tau}= \frac{d \overrightarrow{L}}{dt} =  \overrightarrow{\omega}^{gimbal} \times \mathbf{I} \overrightarrow{\omega}^{disk}
$$

This can be combine on the follwoing equation with s the effects of the disk’s angular acceleration and the interaction (cross product) of the gimbal's angular velocity with the disk's angular momentum, detailing the overall momentum in the system’s frame of reference (A).

$$
\tag{3} \overrightarrow{\mathbf{M}}^{gyro}_A =
\mathbf{I} \cdot
\dot{\overrightarrow{\omega}}^{Disk}_A +
\overrightarrow{\omega}^{Gimbal}_A \times 
(\mathbf{I} \cdot 
\overrightarrow{\omega}^{Disk}_A)
$$

## Variables definition

The controller can be divided into the different objects that it is composed of. The correspondent coordinate spaces and position variables are shown in table 1. The origin of all the spaces is located at the centre of the mass of the flywheel. It is worth noting that since the disk is mounted on the inner gimbal the coordinate space A and B are always in the same orientation. The variables used in this tutorial are: 

$$
\begin{array}{| l | c | c | c |}\hline
\text{Object}        & \text{Space}   & \text{Angle}     & \\\hline
\\[-0.5em] disk      & A              & \rho            & \\ \\[-0.5em]\hline
\\[-0.5em] inner (pitch) & B          & \theta          & \\ \\[-0.5em]\hline
\\[-0.5em] outer (yaw)   & C          & \psi            & \\ \\[-0.5em]\hline
\\[-0.5em] handle    & D              &                 & \\ \\[-0.5em]\hline 
\end{array}
$$

With the subscript  and superscript as follow

$$
\text{variable}^{object}_{space}
$$

For example, in the following equation, we refer to the angular velocity of gimbal2 (the outer gimbal) in the A frame:

$$
\overrightarrow{\omega}{^{outer}_{A}}
$$

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/GyroFrames.png" width="560" alt="Illustration of the HapticWhirl kinematic model showing the A, B, C, and D coordinate frames with axes labeled in red, green, and blue, representing the different components of the system including the disk and gimbals.">
    <br>
    <em>Figure: Coordinate frames of the HapticWhirl kinematic model</em>
</p>

In Equation 3, the angular velocities are denoted as follows:

- $$\overrightarrow{\omega}{^{Gimbal}_{A}}$$ - refers to the combined angular rotation of both gimbals.
- $$\overrightarrow{\omega}{^{Disk}_{A}}$$ - refers to the rotation of the disk and the contribution of both gimbals to the disk's rotational speed.

We decompose the equation from right to left, starting with the angular velocity of the disk in frame A.

## Angular Velocity Disk ($$\overrightarrow{\omega}{^{Disk}_{A}}$$)

This is the sum of the angular velocity of the disk and the contributions to it from the two gimbals in frame A:

$$
\overrightarrow{\omega}{^{Disk}_{A}} = \overrightarrow{\omega}{^{disk}_{A}} + \overrightarrow{\omega}{^{inner}_{A}} + \overrightarrow{\omega}{^{outer}_{A}}
$$

We must apply a rotation matrix to the gimbal; the inner gimbal is rotated from frame B to A, and the outer gimbal from frame C to A:

$$
\overrightarrow{\omega}{^{Disk}_{A}} = 
\begin{pmatrix}
0\\ 
0\\ 
\dot\rho\\
\end{pmatrix} + 
(R_{B \rightarrow A} \cdot \overrightarrow{\omega}{^{inner}_{B}}) +
(R_{C \rightarrow A} \cdot \overrightarrow{\omega}{^{outer}_{C}})  
$$

Disk and inner gimbal share the same coordinate frame hence $$A = B$$. Hence the rotation matrix $$R_{B \rightarrow A}$$ is an identity matrix. The rotation matrix $$R_{C\rightarrow A}$$ rotates over the Y-axis.

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%202.png" width="560" alt="Diagram showing rotation about the Y-axis with coordinates X'(cosB, 0, -sinB), Z'(sinB, 0, cosB), and angles represented, illustrating the rotation matrix R_C to A.">
    <br>
    <em>Figure: Rotation about the Y-axis illustrating the rotation matrix \( R_{C \rightarrow A} \)</em>
</p>

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%203.png" alt="Diagram of rotation matrices and angular velocity vectors applied to the HapticWhirl device, illustrating the transformation from the B frame to the A frame, and the effect of the gimbal's rotation on the disk's angular velocity in the A frame.">
    <br>
    <em>Figure: Rotation matrices and angular velocity vectors in the HapticWhirl device</em>
</p>

The rotation matrix $$R_{A \rightarrow B}$$ is a rotation over the Y-axis, but since the rotation in this case is $$R_{B \rightarrow A}$$ we invert it.

$$
\overrightarrow{\omega}{^{Disk}_{A}}  = 
\begin{pmatrix}0\\ 0\\ \dot\rho\\\end{pmatrix} + \begin{pmatrix}1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1\end{pmatrix} \cdot\begin{pmatrix}0\\ \dot\theta\\ 0\\\end{pmatrix} +\begin{pmatrix}\cos\theta & 0 & -\sin\theta \\ 0 & 1 & 0 \\ \sin\theta & 0 & \cos\theta\end{pmatrix} \cdot\begin{pmatrix}0\\ 0\\ \dot\psi\\\end{pmatrix}
$$

$$
\overrightarrow{\omega}{^{Disk}_{A}}  = 
\begin{pmatrix}
0 &+ &0 &+ &-\dot\psi \sin{\theta} \\ 
0 &+ &\dot\theta &+ &0\\ 
\dot\rho &+ &0 &+ &\dot\psi \cos\theta
\end{pmatrix}
$$

$$
\tag{4} \overrightarrow{\omega}{^{Disk}_{A}}  = \begin{pmatrix}-\dot\psi \sin{\theta} \\ \dot\theta \\ \dot\rho + \dot\psi \cos\theta\end{pmatrix}
$$

The inertia torque matrix of the disk, denoted by $$\mathbf{I}$$, is defined as the inertial tensor representing the moment of inertia about the different axes:

$$
\mathbf{I} = \begin{bmatrix}I_{xx} & I_{xy} & I_{xz} \\ I_{yx} & I_{yy} & I_{yz} \\ I_{zx} & I_{zy} & I_{zz}\end{bmatrix}
$$

However, for a disk, this can be simplified to a [diagonal matrix](https://en.wikipedia.org/wiki/List_of_moments_of_inertia).

$$
\mathbf{I}=\begin{pmatrix}I_{xx}&0&0\\0&I_{yy}&0\\0&0&I_{zz}\end{pmatrix}
$$

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/unnamed.gif" width="560" alt="Illustration of moments of inertia for symmetrical objects like disks, showing the distribution of mass and the resulting inertia around the central axis.">
    <br>
    <em>Figure: Moments of inertia for symmetrical objects</em>
</p>

We can replace $$I_{xx}, I_{yy},\ and\ I_{zz}$$ with the disk's moment of inertia: 

Hence, $$\mathbf{I}$$ becomes:
$$
\tag{5} \mathbf{I}=\begin{pmatrix}\frac{1}{4}MR^2&0&0\\0&\frac{1}{4}MR^2&0\\0&0&\frac{1}{2}MR^2\end{pmatrix}
$$

## Angular Velocity Gimbal ($$\overrightarrow{\omega}{^{Gimbal}_{A}}$$)

The other component in equation (1) corresponds to the sum of the angular velocities of both axes in the gimbal with respect to frame A.

$$
\overrightarrow{\omega}{^{Gimbal}_{A}} =  R_{B \rightarrow A} \cdot \overrightarrow{\omega}{^{inner}_{B}} + R_{C \rightarrow A} \cdot \overrightarrow{\omega}{^{outer}_{C}} 
$$

$$
\overrightarrow{\omega}{^{Gimbal}_{A}} = \begin{pmatrix} 1 & 0 & 0 \\  0 & 1 & 0 \\   0 & 0 & 1 \end{pmatrix} \cdot
  \begin{pmatrix}0\\ \dot{\theta}\\ 0\\\end{pmatrix} +
 \begin{pmatrix} \cos{\theta} & 0 & -\sin{\theta} \\  0 & 1 & 0 \\   \sin{\theta} & 0 & \cos{\theta}  \end{pmatrix} \cdot
  \begin{pmatrix}0\\ 0\\ \dot{\psi}\\\end{pmatrix}
$$

$$
\tag{6}\overrightarrow{\omega}{^{Gimbal}_{A}} = 
\begin{pmatrix}
{-\dot\psi\sin\theta}\\
\dot\theta \\ 
{\dot\psi\cos\theta}\\
\end{pmatrix}
$$

## Angular Acceleration Disk ($$\dot{\overrightarrow{\omega}}{^{Disk}_{A}}$$)
Derivative of equation (4) applying the chain rule:

$$
\tag{7}\dot{\overrightarrow{\omega}}{^{Disk}_{A}} = \begin{pmatrix}{-\ddot\psi\sin\theta} - {\dot\psi\dot\theta\cos\theta}\\\ddot\theta\\ {\ddot\rho + \ddot\psi\cos\theta} - {\dot\psi\dot\theta\sin\theta}\\\end{pmatrix}
$$

## Equation Expanded

The equation for the gyroscopic torque can be expanded by considering the inertial properties of the disk and the angular velocities involved. We apply the inertia tensor to the angular acceleration vector, then add the cross product of the angular velocity vector and the angular momentum vector. This approach takes into account the non-commutative nature of the cross product and matrix multiplication in rotational dynamics. The expanded form of equation (1) is as follows:

$$
\tag{8}\overrightarrow{\mathbf{M}}^{gyro}_{A} = 
\begin{pmatrix}
\frac{1}{4}MR^2 & 0 & 0 \\
0 & \frac{1}{4}MR^2 & 0 \\
0 & 0 & \frac{1}{2}MR^2
\end{pmatrix}
\cdot
\begin{pmatrix}
-\ddot\psi\sin\theta - \dot\psi\dot\theta\cos\theta \\
\ddot\theta \\
\ddot\rho + \ddot\psi\cos\theta - \dot\psi\dot\theta\sin\theta
\end{pmatrix}
+
\begin{pmatrix}
-\dot\psi\sin\theta \\
\dot\theta \\
\dot\psi\cos\theta
\end{pmatrix}
\times
\left(
\begin{pmatrix}
\frac{1}{4}MR^2 & 0 & 0 \\
0 & \frac{1}{4}MR^2 & 0 \\
0 & 0 & \frac{1}{2}MR^2
\end{pmatrix}
\cdot
\begin{pmatrix}
-\dot\psi\sin\theta \\
\dot\theta \\
\dot\rho + \dot\psi\cos\theta
\end{pmatrix}
\right)
$$

We can combine the members of the equation by  grouping the different equations to solve in three steps:

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%204.png" width="400" alt="Diagram showing the groupings of gyroscopic torque equations with labeled components for clearer step-by-step calculation in the HapticWhirl project.">
</p>

First we replace the diagonal values in $$I$$  with $$I_x , I_y \text{ and } I_z$$ , where $$I_x = I_y$$ . 

$$
\mathbf{I_x} = \mathbf{I_y} = \frac{1}{4}MR^2  \qquad
\mathbf{I_z} = \frac{1}{2}MR^2
$$

Solving group 1: 

$$
\begin{pmatrix}
\mathbf{I_x} \left( -\ddot\psi \sin\theta - \dot\psi \dot\theta \cos\theta \right) \\
\mathbf{I_x} \ddot\theta \\ 
\mathbf{I_z} \left( \ddot\rho + \ddot\psi \cos\theta - \dot\psi \dot\theta \sin\theta \right) \\
\end{pmatrix}
$$

Solving group 2:

$$
\begin{pmatrix}
\mathbf{I_x} \left( -\dot\psi \sin\theta \right) \\
\mathbf{I_x} \dot\theta \\
\mathbf{I_z} \left( \dot\rho + \dot\psi \cos\theta \right) \\
\end{pmatrix}
$$

Solving group 3:

$$
\begin{pmatrix}
-\dot\psi \sin\theta \\
\dot\theta \\
\dot\psi \cos\theta \\
\end{pmatrix} \times
\begin{pmatrix}
\mathbf{I_x} \left( -\dot\psi \sin\theta \right) \\
\mathbf{I_x} \dot\theta \\
\mathbf{I_z} \left( \dot\rho + \dot\psi \cos\theta \right) \\
\end{pmatrix}
$$

$$
\begin{pmatrix}i&j&k\\ -\dot\psi \sin \theta &\dot\theta &\dot\psi \cos \theta \\ {\bf{I_x}}\left(-\dot\psi \sin \theta \right)&{\bf{I_{x}}}\dot\theta &{\bf{I_z}}\left(\dot\rho +\dot\psi \cos \theta \right)\end{pmatrix}=
$$

$$
\begin{pmatrix}
\dot\theta \mathbf{I_z}(\dot\rho + \dot\psi \cos \theta) - \dot\psi \cos \theta \mathbf{I_z}(\dot\rho + \dot\psi \cos \theta) \\
(\dot\psi \cos \theta)(-\mathbf{I_x} \dot\psi \sin \theta) - (\mathbf{I_z}(\dot\rho + \dot\psi \cos \theta)(-\dot\psi \sin \theta)) \\
-\dot\psi \sin \theta \mathbf{I_x} \dot\theta - \mathbf{I_x}(-\dot\psi \sin \theta) \dot\theta
\end{pmatrix}
$$

Grouping the terms group 3: 

$$
\begin{pmatrix}
-\mathbf{I_x} \dot\theta \dot\psi \cos \theta + \mathbf{I_z} \dot\theta \dot\rho \\
-\mathbf{I_x} \dot\psi^2 \cos \theta \sin \theta + \mathbf{I_z} \dot\psi^2 \cos \theta \sin \theta + \mathbf{I_z} \dot\rho \dot\psi \sin \theta \\
0
\end{pmatrix}
$$

This leaves us with the equation:

$$
\overrightarrow{\mathbf{M}}^{gyro}_{A} = 
\begin{pmatrix}
\mathbf{I_x}(-\ddot\psi\sin\theta - \dot\psi\dot\theta\cos\theta) \\
\mathbf{I_x}\ddot\theta \\ 
\mathbf{I_z}(\ddot\rho + \ddot\psi\cos\theta - \dot\psi\dot\theta\sin\theta)
\end{pmatrix} + 
\begin{pmatrix}
-\mathbf{I_x} \dot\theta \dot\psi \cos\theta + \mathbf{I_z} \dot\theta \dot\psi \cos\theta + \mathbf{I_z} \dot\theta \dot\rho \\
-\mathbf{I_x} \dot\psi^2 \cos\theta \sin\theta + \mathbf{I_z} \dot\psi^2 \cos\theta \sin\theta + \mathbf{I_z} \dot\rho \dot\psi \sin\theta \\
0
\end{pmatrix}
$$

Combining terms, the gyroscopic moment becomes:

$$
\overrightarrow{\mathbf{M}}^{gyro}_{A} =  
\begin{pmatrix}
\mathbf{I_x} (-\dot\theta \dot\psi \cos\theta - \ddot\psi \sin\theta) - \mathbf{I_x} \dot\theta \dot\psi \cos\theta + \mathbf{I_z} \dot\theta \dot\psi \cos\theta + \mathbf{I_z} \dot\theta \dot\rho \\
\mathbf{I_x} \ddot\theta - \mathbf{I_x} \dot\psi^2 \cos\theta \sin\theta + \mathbf{I_z} \dot\psi^2 \cos\theta \sin\theta + \mathbf{I_z} \dot\rho \dot\psi \sin\theta \\
\mathbf{I_z} (\ddot\rho + \ddot\psi \cos\theta - \dot\theta \dot\psi \sin\theta)
\end{pmatrix}
$$

To further simplify the equation, we substitute the $$\mathbf{I}$$ terms across the equation:

$$
\text{If} \quad \mathbf{I} = MR^2, \quad \mathbf{I_x} = \frac{1}{4} MR^2 = \frac{1}{4} \mathbf{I}, \quad \mathbf{I_z} = \frac{1}{2}MR^2 = \frac{1}{2} \mathbf{I}
$$

$$
\overrightarrow{\mathbf{M}}^{gyro}_{A} =  \begin{pmatrix}\frac{1}{4}\mathbf{I} (-\dot\theta  \dot\psi  \cos  \theta -\ddot\psi  \sin  \theta ) -\frac{1}{4}\mathbf{I} \dot\theta  \dot\psi  \cos  \theta +\frac{1}{2}\mathbf{I} \dot\theta  \dot\psi  \cos  \theta  + \frac{1}{2}\mathbf{I} \dot\theta  \dot\rho\\\frac{1}{4}\mathbf{I} \ddot\theta -\frac{1}{4}\mathbf{I} \dot\psi ^2 \cos  \theta  \sin  \theta +\frac{1}{2}\mathbf{I} \dot\psi ^2 \cos  \theta  \sin  \theta  +\frac{1}{2}\mathbf{I} \dot\rho  \dot\psi  \sin  \theta  \\\frac{1}{2}\mathbf{I} (\ddot\rho +\ddot\psi  \cos  \theta -\dot\theta  \dot\psi  \sin  \theta )\end{pmatrix}
$$

We collect common terms and simplify the equation. The final equation for the momentum of the flywheel in our VR controller is:

$$
\tag{9}\overrightarrow{\mathbf{M}}^{gyro}_{A} =\begin{pmatrix}\frac{1}{4} \mathbf{I} (2 \dot\theta \dot\rho -\ddot\psi \sin \theta ) \\ \frac{1}{4} \mathbf{I} (\ddot\theta + \dot\psi  \sin \theta  (2 \dot\rho + \dot\psi  \cos \theta )) \\ \frac{1}{2} \mathbf{I} (\ddot\rho + \ddot\psi  \cos \theta - \dot\theta  \dot\psi  \sin \theta )\end{pmatrix}
$$

## Torque applied on the handle

The output of equation (9) represents the momentum in frame A. To express this relative to the handle, we need to translate it to the corresponding system of reference by applying the appropriate rotation matrix:

$$
\overrightarrow{\mathbf{M}}^{gyro}_{D} =  R_{A \rightarrow D} \cdot \overrightarrow{\mathbf{M}}^{gyro}_{A} = R_{C \rightarrow D} \cdot R_{A \rightarrow C} \cdot \overrightarrow{\mathbf{M}}^{gyro}_{A}
$$

The transformation matrix from A to D is composed of two rotation matrices. The first step from A to C involves a rotation about the Y-axis, and the second step from C to D involves a rotation about the Z-axis:

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%205.png" width="560" alt="Diagram showing the rotation from frame A to C around the Y-axis, and from frame C to D around the Z-axis, demonstrating the orientation change for the handle in the HapticWhirl device.">
    <br>
    <em>Figure: Rotation from frame A to frame D through intermediate frame C</em>
</p>

$$
R_{A \rightarrow C} =
\begin{pmatrix}
\color{red}\cos\theta & 0 & \color{blue}\sin\theta \\
0 & 1 & 0 \\
\color{red}-\sin\theta & 0 & \color{blue}\cos\theta \\
\end{pmatrix}\\
$$

Rotation over the Z-axis:

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%206.png" width="560" alt="Diagram showing the rotation matrix for rotation about the Z-axis, used to transform coordinates from frame C to frame D in the HapticWhirl device.">
    <br>
    <em>Figure: Rotation matrix for Z-axis transformation from frame C to D</em>
</p>

$$
R_{C \rightarrow D} = 
\begin{pmatrix}
\color{red}\cos\psi & \color{green}-\sin\psi & 0 \\
\color{red}\sin\psi & \color{green}\cos\psi & 0 \\
0 & 0 & 1 \\
\end{pmatrix}\\
$$

By combining both rotation matrices $$R_{A\rightarrow D} = R_{A \rightarrow C} \cdot  R_{C \rightarrow D}$$ we obtain the following rotation matrix:

$$
\tag{10} R_{A \rightarrow D} =  \begin{pmatrix}\cos\theta\cos\psi & -\sin\psi  &\sin\theta\cos\psi\\\cos\theta\sin\psi  & \cos\psi  &\sin\theta\sin\psi\\-\sin\theta&0&\cos\theta\\\end{pmatrix}\\
$$

Finally, the torque applied on the handle is computed by combining the results from equations (7) and (8) as follows:

$$
\overrightarrow{\mathbf{M}}^{gyro}_{D} =  R_{A \rightarrow D} \cdot \overrightarrow{\mathbf{M}}^{gyro}_{A}
$$

# Solving for the different moments

If we have a given torque and we want to replicate this on the controller we solve using the acceleration of the gimbal axis. To solve the for the different variables we usXe equation 9.

## Solving for $$\ddot\psi$$

Yaw acceleration is part of the $$X \text{ and } Z$$  axis. We solve first using the $$X$$component: 

$$
\overrightarrow{\bf {M}}^{gyro}_{Ax} = \frac{1}{4} {\bf{I}} (2 \dot\theta \dot\rho -\ddot\psi \sin \theta )
$$

Rearranging to solve for $$\ddot\psi$$:

$$
2\dot\theta \dot\rho - \ddot\psi\sin\theta = \frac{4\overrightarrow{\bf {M}}^{gyro}_{Ax}}{{\bf{I}}}
$$

$$-\ddot\psi\sin\theta = \frac{4\overrightarrow{\bf {M}}^{gyro}_{Ax}}{{\bf{I}}} - 2\dot\theta\dot\rho$$

$$\ddot\psi = \frac{2\dot\theta\dot\rho}{\sin\theta} - \frac{4\overrightarrow{\bf {M}}^{gyro}_{Ax}}{{\bf{I}}\sin\theta}$$

Using the Z component

$$
\overrightarrow{\bf {M}}{^{gyro}_{Az}} =\frac{1}{2}{\bf{I}} \left(\ddot\rho +\ddot\psi  \cos  \theta -\dot\theta  \dot\psi  \sin  \theta \right)
$$

$$
\frac{2 \overrightarrow{\bf {M}}{^{gyro}_{Az}} }{{\bf{I}}} = \ddot\rho +\ddot\psi  \cos  \theta -\dot\theta  \dot\psi  \sin  \theta 
$$

$$
- \ddot\rho +\frac{2 \overrightarrow{\bf {M}}{^{gyro}_{Az}} }{{\bf{I}}} +\dot\theta  \dot\psi  \sin  \theta  = \ddot\psi  \cos  \theta 
$$

$$
\ddot\psi =\frac{2 \overrightarrow{\bf {M}}{^{gyro}_{Az}} }{{\bf{I}}\cos  \theta }-\frac{\ddot\rho}{\cos  \theta }  +\dot\theta  \dot\psi  \tan  \theta 
$$

## Solving for $$\ddot\theta$$

$$
\frac{1}{4} {\bf{I}} (\ddot\theta +\dot\psi  \sin \theta  (2 \dot\rho +\dot\psi  \cos \theta ))
$$

$$
\overrightarrow{\bf {M}}{^{gyro}_{Ay}}=\frac{1}{4}{\bf{I}} \left(\dot\psi   \sin\theta  (\dot\psi  \cos \theta +2 \dot\rho )+\ddot\theta \right)
$$

$$
\frac{4\overrightarrow{\bf {M}}{^{gyro}_{Ay}}}{{\bf{I}}}=\dot\psi   \sin\theta  (\dot\psi  \cos \theta +2 \dot\rho )+\ddot\theta 
$$

$$
\ddot\theta = \frac{4\overrightarrow{\bf {M}}{^{gyro}_{Ay}}}{{\bf{I}}} -\dot\psi   \sin\theta  (\dot\psi  \cos \theta +2 \dot\rho )
$$

## Solving for $$\dot\theta$$

Using the X component, we can solve for $$\dot\theta$$ :

$$
\overrightarrow{\bf {M}}{^{gyro}_{Ax}}={\bf{I}}  \left(\frac{\dot\theta  \dot\rho }{2}-\frac{1}{4} \ddot\psi  (\sin  \theta )\right) 
$$

$$
\frac{\overrightarrow{\bf {M}}{^{gyro}_{Ax}}}{{\bf{I}} } = \frac{\dot\theta \dot\rho }{2}-\frac{1}{4} \ddot\psi (\sin \theta )
$$

$$
\frac{\overrightarrow{\bf {M}}{^{gyro}_{Ax}}}{{\bf{I}}  } +\frac{1}{4} \ddot\psi  (\sin  \theta ) = \frac{\dot\theta  \dot\rho }{2}
$$

$$
\dot\theta = \frac{2\overrightarrow{\bf {M}}{^{gyro}_{Ax}}}{{\bf{I}}\dot\rho  } +\frac{\ddot\psi  (\sin  \theta)}{2\dot\rho}  
$$

Using the Z component: 

$$
\overrightarrow{\bf {M}}{^{gyro}_{Az}}=\frac{1}{2} {\bf{I}} (\ddot\rho +\ddot\psi  (\cos  \theta )-\dot\psi  \dot\theta  (\sin  \theta ))
$$

$$
\ddot\rho +\ddot\psi  (\cos  \theta )-\dot\psi \dot\theta  (\sin  \theta )=\frac{2 \overrightarrow{\bf {M}}{^{gyro}_{Az}}}{{\bf{I}}}
$$

$$
\tag{x}-\dot\psi \dot\theta  (\sin  \theta )=\frac{2 \overrightarrow{\bf {M}}{^{gyro}_{Az}}}{{\bf{I}}} + (-\ddot\rho -\ddot\psi  (\cos  \theta ))
$$

$$
\text{Divide }\text{both }\text{sides }\text{by }-\psi  (\theta  \sin ):
$$

$$
 \dot\theta  = \cot(\theta)-\frac{2 \overrightarrow{\bf {M}}{^{gyro}_{Az}}\csc(\theta)}{{\bf{I}}\dot\psi} + \frac{\ddot\rho \csc(\theta)}{\dot\psi}
$$

## Solving for $$\dot\psi$$

Using the Y component, we solve for $$\dot\psi$$:

$$
\ddot\theta +\dot\psi  (\sin  \theta ) (2 \dot\rho +\dot\psi  (\cos  \theta ))=\frac{4 \overrightarrow{\bf {M}}{^{gyro}_{Ay}}}{\bf{I}}
$$

$$
\ddot\theta +2 \dot\rho  \dot\psi  (\sin  \theta )+\dot\psi ^2 (\cos  \theta ) (\sin  \theta )=\frac{4 \overrightarrow{\bf {M}}{^{gyro}_{Ay}}}{\bf{I}}
$$

$$
\text{divide by } \sin(\theta)\cos(\theta);
$$

$$
\\ \dot\psi ^2+2 \dot\rho  \dot\psi  (\sec  \theta )+\ddot\theta  (\csc  \theta ) (\sec  \theta )=\frac{4 \overrightarrow{\bf {M}}{^{gyro}_{Ay}} (\csc \theta) ( \sec\theta )}{{\bf{I}}}
$$

$$
\dot\psi ^2+2 \dot\rho  \dot\psi  (\sec  \theta )=\frac{4 \overrightarrow{\bf {M}}{^{gyro}_{Ay}} (\csc  \theta ) (\sec  \theta )}{{{\bf{I}}}}-\ddot\theta  (\csc  \theta ) (\sec  \theta )
$$

$$
\text{Add }\rho ^2 \left(\theta  \sec ^2\right) \text{to }\text{both }\text{sides}:
$$

$$
\dot\psi ^2+2 \dot\rho  \dot\psi  (\sec  \theta )+\dot\rho ^2 \left(\sec ^2 \theta \right)=\\\frac{4 \overrightarrow{\bf {M}}{^{gyro}_{Ay}} (\csc  \theta ) (\sec  \theta )}{{{{\bf{I}}}}}-\ddot\theta  (\csc  \theta ) (\sec  \theta )+\dot\rho ^2 \left(\sec ^2 \theta \right)
$$

$$
(\psi +\rho  (\sec  \theta ))^2 = \frac{4 \overrightarrow{\bf {M}}{^{gyro}_{Ay}} (\csc  \theta ) (\sec  \theta )}{{{{\bf{I}}}}}-\ddot\theta  (\csc  \theta ) (\sec  \theta )+\dot\rho ^2 \left(\sec ^2 \theta \right)
$$

$$
\dot\psi +\dot\rho  (\sec  \theta )=\sqrt{\frac{4 \overrightarrow{\bf {M}}{^{gyro}_{Ay}} (\csc  \theta ) (\sec  \theta )}{x}-\ddot\theta  (\csc  \theta ) (\sec  \theta )+\dot\rho ^2 \left(\sec ^2 \theta \right)}
$$

$$
\dot\psi =\sqrt{\frac{4 \overrightarrow{\bf {M}}{^{gyro}_{Ay}}  \csc (\theta ) \sec (\theta )}{{\bf{I}}}-\ddot\theta  \csc (\theta ) \sec (\theta )+\dot\rho ^2 \sec (\theta )^2}-\dot\rho  (\sec  \theta ) \qquad \text{ or } \\ \dot\psi =-\dot\rho  (\sec  \theta )-\sqrt{\frac{4 \overrightarrow{\bf {M}}{^{gyro}_{Ay}}  \csc (\theta ) \sec (\theta )}{{\bf{I}}}-\ddot\theta  \csc (\theta ) \sec (\theta )+\dot\rho ^2 \sec (\theta )^2}
$$

Using the Z component, follow up from equation x:

$$
-\dot\psi \dot\theta  (\sin  \theta )=\frac{2 \overrightarrow{\bf {M}}{^{gyro}_{Az}}}{{\bf{I}}} + (-\ddot\rho -\ddot\psi  (\cos  \theta ))
$$

$$
\text{Divide }\text{both }\text{sides }\text{by }-\theta  (\theta  \sin ):
$$

$$
\dot\psi =\frac{\ddot\psi  (\cot  \theta )}{\dot\theta }-\frac{2 \overrightarrow{\bf {M}}{^{gyro}_{Az}} (\csc  \theta )}{{\bf{I}} \dot\theta }+\frac{\dot\rho  (\csc  \theta )}{\dot\theta }
$$

## Solving equation 1

solving on equation 1 for $$\ddot\theta \text{ and } \ddot\psi$$  , solving the $$X$$  component first: 

$$
m_x ={\bf{I_x}}\left({-\ddot\psi\sin\theta}-{\dot\psi\dot\theta\cos\theta}\right) \\ 
$$

$$
\tag{x}\frac{m_x}{{\bf{I_x}}} ={-\ddot\psi\sin\theta}-{\dot\psi\dot\theta\cos\theta}
$$

$$
\frac{m_x}{{\bf{I_x}}}+{\dot\psi\dot\theta\cos\theta} ={-\ddot\psi\sin\theta}
$$

$$
\tag{x}\ddot\psi={\dot\psi\dot\theta\cot\theta}\frac{m_x\csc\theta}{{\bf{I_x}}}
$$

Solving for $$\dot\theta \text{ and } \dot\psi$$, we continue from equation x above:

$$
\frac{m_x}{{\bf{I_x}}}{+\ddot\psi\sin\theta} ={\dot\psi\dot\theta\cos\theta}
$$

$$
\dot\psi =\frac{m_x sec\theta}{{\bf{I_x}}\dot\theta}{+\frac{\ddot\psi \tan\theta}{\dot\theta}} 
$$

$$
\dot\theta =\frac{m_x sec\theta}{{\bf{I_x}}\dot\psi}{+\frac{\ddot\psi \tan\theta}{\dot\psi}} 
$$

Solving the $$Y$$ component:

$$
m_y ={\bf{I_y}}\ddot\theta
$$

$$
\ddot\theta = \frac{m_y}{{\bf{I_y}}}
$$

Solving the $$Z$$  component: 

$$
m_z = {\bf{I_z}}\left({\ddot\rho +\ddot\psi\cos\theta}-{\dot\psi\dot\theta\sin\theta}\right)
$$

When the controller is running the disk is rotating at constant speed, hence $$\ddot\rho$$ is zero. Hence:

$$
\frac{m_z}{{\bf{I_z}}}+\dot\psi\dot\theta\sin\theta = \ddot\psi\cos\theta
$$

$$
\ddot\psi = \frac{m_z sec\theta}{{\bf{I_z}}}+\dot\psi\dot\theta\tan\theta
$$

Solving for $$\dot\theta \text{ and } \dot\psi$$:

$$
\frac{m_z}{{\bf{I_z}}}-\ddot\psi\cos\theta = \dot\psi\dot\theta\sin\theta 
$$

$$
\dot\psi = \frac{\ddot\psi \cot\theta}{\dot\theta} -\frac{m_z \csc\theta}{{\bf{I_z}} \dot\theta}
$$

$$
\dot\theta= \frac{\ddot\psi \cot\theta}{\dot\psi} -\frac{m_z \csc\theta}{{\bf{I_z}} \dot\psi}
$$

