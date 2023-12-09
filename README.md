<script type="text/javascript" id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

Welcome to this guide for constructing and operating a kinesthetic haptic controller that harnesses the dynamic properties of a flywheel with dual axes of rotation. This page serves as a one-stop resource, offering all the essential files, codes, and  instructions necessary to create your own gyroscope-based haptic device.

For a head start, we've provided a fully-realized 3D model, ready for 3D printing, accessible via the Fusion website. You can explore the model in detail through the embedded frame below or download it from the dropdown menu:

<div style="text-align:center;">
<iframe width="800" height="450" src="https://myhub.autodesk360.com/ue2901c7f/g/shares/SHd38bfQT1fb47330c99796b3271601d0da5" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

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

The actuation of the flywheel and the gimbal creates two moments that result in torque.

### Momentum Wheel

<p align="center">
<img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled.png" width="560">
</p>

(a) Momentum wheel

The momentum of the flywheel is perpendicular to the rotation plane of the flywheel. The torque output can be determined using the following equation:

$$
\tag{1}\overrightarrow{\tau}= \frac{d \overrightarrow L}{dt} =\mathbf{I}  \frac{d}{dt} \overrightarrow\omega^{disk} = \mathbf{I}  \dot{\overrightarrow{\omega}}{^{disk}_{}}
$$

$$
\overrightarrow{\tau} = \frac{d\overrightarrow{L}}{dt} = \mathbf{I} \frac{d}{dt}\overrightarrow{\omega}^{\text{disk}} = \mathbf{I} \dot{\overrightarrow{\omega}}^{\text{disk}}
$$

This equation indicates that the torque output is the product of the flywheel's moment of inertia and the rate of change of its angular velocity.


## Need to edit from here forward 

### Steered Momentum Wheel

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%201.png" width="560" alt="(b) Steered momentum wheel">
</p>


(b) Steered momentum wheel

The steered momentum wheel refers to the output momentum generated when the gimbal axis is also actuated. 

$$
\tag{2}\overrightarrow{\tau}= \frac{d \overrightarrow L}{dt} =  \overrightarrow\omega^{gimbal} \times \mathbf{I} \overrightarrow\omega^{disk} 
$$

The combination of equations 1. and 2. describes the momentum caused by the disk in its frame of reference (A), considering both the angular acceleration of the disk and the cross product of the gimbal's angular velocity with the disk's angular momentum. 

$$
\tag{3} {\overrightarrow{\bf {M}}{^{gyro}_{A}} =
{\bf{I}} \cdot
\dot{\overrightarrow{\omega}}{^{Disk}_{A}}+
\overrightarrow{\omega}{^{Gimbal}_{A}} \times 
({\bf{I}} \cdot 
{\overrightarrow{\omega}}{^{Disk}_{A}}) }
$$

## Variables definition

The controller can be divided into the different objects that it is composed of. The correspondent coordinate spaces and position variables are shown in table 1. The origin of all the spaces is located at the centre of the mass of the flywheel. It is worth noting that since the disk is mounted on the inner gimbal the coordinate space A and B are always in the same orientation. The variables used in this tutorial are: 

$$
\begin{array}{| l | c | c | c |}\hline
\text{Object}        & \text{Space}   & \text{Angle}     \\\hline
\\[-0.5em] disk      & A              & \rho            \\ \\[-0.5em]\hline
\\[-0.5em] inner (pitch) & B          & \theta          \\ \\[-0.5em]\hline
\\[-0.5em] outer (yaw)   & C          & \psi            \\ \\[-0.5em]\hline
\\[-0.5em] handle    & D              &                 \\ \\[-0.5em]\hline 
\end{array}
$$

With the subscript  and superscript as follow

$$
\text{variable}{^{objcet}_{space}} 
$$

For example in the following equation, we refer to the angular velocity of the gimbal2 (outer gimbal ) on the A space. 

$$
\overrightarrow\omega{^{outer}_{A}} 
$$

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/GyroFrames.png" width="560" alt="HapticWhirl - GyroFrames">
</p>

Equation 3 angular velocities $$\omega$$:

- $$\overrightarrow{\omega}{^{Gimbal}_{A}}$$ - refers to the sum angular rotation of both gimbals.
- $$\overrightarrow{\omega}{^{Disk}_{A}}$$ - refers to the rotation of the disk and the contribution of both gimbals to the speed of the disk.

We decompose the equation from right to left, starting with the angular velocity of the disk in frame A.

## Angular Velocity Disk($$\overrightarrow{\omega}{^{Disk}_{A}}$$ )

This is the sum of the angular velocity of the disk and the contribution to it of the two gimbals in frame A:  

$$
\overrightarrow{\omega}{^{Disk}_{A}} = \overrightarrow{\omega}{^{disk}_{A}} + \overrightarrow{\omega}{^{inner}_{A}} + \overrightarrow{\omega}{^{outer}_{A}}
$$

We have to apply a rotation matrix to the gimbal, the inner gimbal is rotated from frame B→A and the outer matrix rotates from frame C → A:

$$
\overrightarrow{\omega}{^{Disk}_{A}} = 

\begin{pmatrix}
0\\ 
0\\ 
\dot\rho\\
\end{pmatrix} + 
(R_{B \rightarrow A} \ \cdot \overrightarrow{\omega}{^{inner}_{B}}) +
(R_{C \rightarrow A} \ \cdot \overrightarrow{\omega}{^{outer}_{C}})  
$$

Disk and inner gimbal share the same coordinate frame hence  $$A = B$$.  Hence the rotation matrix $$R_{B \rightarrow A}$$ is an identity matrix. The rotation matrix $$R_{C\rightarrow A}$$  rotates over the Y-axis. 

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%202.png" width="560" alt="HapticWhirl Image">
</p>

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%203.png" alt="HapticWhirl Image 3">
</p>

The rotation matrix $$R_{A \rightarrow B}$$ is a rotation over the Y axis, but since the rotation in this case is $$R_{B \rightarrow A}$$ we invert it. 

$$
\overrightarrow{\omega}{^{Disk}_{A}}  = 
\begin{pmatrix}0\\ 0\\ \dot\rho\\\end{pmatrix} + \begin{pmatrix}1 & 0 & 1 \\ 0 & 1 & 0 \\ 0 & 0 & 1\end{pmatrix} \cdot\begin{pmatrix}0\\ \dot\theta\\ 0\\\end{pmatrix} +\begin{pmatrix}\cos\theta & 0 & -sin\theta \\ 0 & 1 & 0 \\ \sin\theta & 0 & \cos\theta\end{pmatrix} \cdot\begin{pmatrix}0\\ 0\\ \dot\psi\\\end{pmatrix}
$$

$$
\overrightarrow{\omega}{^{Disk}_{A}}  = 
\begin{pmatrix}
&0 &+ &0 &+ & -\dot\psi \sin{\theta} \\ 
&0 &+ &\dot\theta  &+ &0\\ 
&\dot\rho &+  &0 &+  &\dot\psi \cos\theta
\end{pmatrix}
$$

$$
\tag{4} \overrightarrow{\omega}{^{Disk}_{A}}  = \begin{pmatrix}-\dot\psi \sin{\theta} \\ \dot\theta \\ \dot\rho + \dot\psi \cos\theta\end{pmatrix}
$$

Disk Inertia Torque Matrix ($$I$$)

The moment component $$\mathbf{I}$$ refers to the Inertial tensor, representing the moment of inertia about the different axes :

$$
\mathbf{I} = \begin{bmatrix}I_{xx}&I_{xy}&I_{xz}\\I_{yx}&I_{yy}&I_{yz}\\I_{zx}&I_{zy}&I_{zz}\end{bmatrix}
$$

However, for a disk this can be simplified to a [diagonal matrix](https://en.wikipedia.org/wiki/List_of_moments_of_inertia). 

$$
\mathbf{I}=\begin{pmatrix}I_{xx}&0&0\\0&I_{yy}&0\\0&0&I_{zz}\end{pmatrix}
$$

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/unnamed.gif" width="560" alt="HapticWhirl Animation">
</p>

We can replace $$I_{xx}, I_{yy}\ \ and\ \ I_{zz}$$ with the disk moment of inertia: 

hence $$I$$ becomes:

$$
\tag{5} \mathbf{I}=\begin{pmatrix}\frac{1}{4}MR^2&0&0\\0&\frac{1}{4}MR^2&0\\0&0&\frac{1}{2}MR^2\\\end{pmatrix}
$$

## Angular Velocity Gimbal($$\overrightarrow{\omega}{^{Gimbal}_{A}}$$)

The other component on equation 1. is the correspondent to the sum of the angular velocity of both of the axis in the gimbal with respect to frame A. 

$$
\overrightarrow{\omega}{^{Gimbal}_{A}} =  R_{B \rightarrow A} \cdot \overrightarrow\omega{^{inner}_{B}} +R_{C \rightarrow A} \cdot \overrightarrow\omega{^{outer}_{C}} 
$$

$$
\overrightarrow{\omega}{^{Gimbal}_{A}} = \begin{pmatrix} 1 & 0 & 0 \\  0 & 1 & 0 \\   0 & 0 & 1\end{pmatrix} \cdot
  \begin{pmatrix}0\\ \dot\theta\\ 0\\\end{pmatrix} +
 \begin{pmatrix} \cos\theta & 0 & -sin\theta \\  0 & 1 & 0 \\   \sin\theta & 0 & \cos\theta  \end{pmatrix} \cdot
  \begin{pmatrix}0\\ 0\\ \dot\psi\\\end{pmatrix}
$$

$$
\tag{6}\overrightarrow{\omega}{^{Gimbal}_{A}} = 
\begin{pmatrix}
{-\dot\psi\sin\theta}\\
\dot\theta \\ 
{\dot\psi\cos\theta}\\
\end{pmatrix}
$$

## Angular Acceleration Disk($$\dot{\overrightarrow{\omega}}{^{Disk}_{A}}$$)

Derivative of equation 4 applying the chain rule: 

$$
\tag{7}\dot{\overrightarrow{\omega}}{^{Disk}_{A}} = \begin{pmatrix}{-\ddot\psi\sin\theta}-{\dot\psi\dot\theta\cos\theta}\\\ddot\theta\\ {\ddot\rho +\ddot\psi\cos\theta}-{\dot\psi\dot\theta\sin\theta}\\\end{pmatrix}
$$

## Equation Expanded

We can expand equation 1 as follow: 

$$
\tag{8}\overrightarrow{\bf {M}}{^{gyro}_{A}} = \begin{pmatrix}\frac{1}{4}MR^2&0&0\\0&\frac{1}{4}MR^2&0\\0&0&\frac{1}{2}MR^2\\\end{pmatrix}\cdot\begin{pmatrix}{-\ddot\psi\sin\theta}-{\dot\psi\dot\theta\cos\theta}\\\ddot\theta\\ {\ddot\rho +\ddot\psi\cos\theta}-{\dot\psi\dot\theta\sin\theta}\\\end{pmatrix}+\\\begin{pmatrix}  {-\dot\psi\sin\theta} \\ \dot\theta\\  {\dot\psi\cos\theta} \\\end{pmatrix}\times \left(\begin{pmatrix}\frac{1}{4}MR^2&0&0\\0&\frac{1}{4}MR^2&0\\0&0&\frac{1}{2}MR^2\\\end{pmatrix}\cdot\begin{pmatrix}{-\dot\psi\sin\theta}\\\dot\theta\\ {\dot\rho +\dot\psi\cos\theta}\\\end{pmatrix}\right)
$$

We can combine the member of both sides of the equation, for that we first group the different equations to do it on three steeps: 

![HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%204.png](HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%204.png)

 First we replace the diagonal values in $I$  with $I_x , I_y \text{ and } I_z$ , where $I_x = I_y$ . 

$$
{\bf{I_x}}= {\bf{I_y}} =  \frac{1}{4}MR^2  \qquad
{\bf{I_z}}= \frac{1}{2}MR^2

$$

Solving group 1: 

$$
\begin{pmatrix}
{\bf{I_x}}\left({-\ddot\psi\sin\theta}-{\dot\psi\dot\theta\cos\theta}\right) \\
{\bf{I_x}}\ddot\theta \\ 
{\bf{I_z}}\left({\ddot\rho +\ddot\psi\cos\theta}-{\dot\psi\dot\theta\sin\theta}\right) \\
\end{pmatrix}
$$

Solving group 2 :

$$
\begin{pmatrix}{\bf{I_x}}\left({-\dot\psi\sin\theta}\right)\\{\bf{I_x}}\dot\theta\\ {\bf{I_z}}\left({\dot\rho +\dot\psi\cos\theta}\right)\\\end{pmatrix}
$$

Solving group 3: 

$$
\begin{pmatrix}  {-\dot\psi\sin\theta} \\ \dot\theta\\  {\dot\psi\cos\theta} \\\end{pmatrix} \times \begin{pmatrix}{\bf{I_x}}\left({-\dot\psi\sin\theta}\right)\\{\bf{I_x}}\dot\theta\\ {\bf{I_z}}\left({\dot\rho +\dot\psi\cos\theta}\right)\\\end{pmatrix}
$$

$$
\begin{pmatrix}i&j&k\\ -\dot\psi \sin \theta &\dot\theta &\dot\psi \cos \theta \\ {\bf{I_x}}\left(-\dot\psi \sin \theta \right)&{\bf{I_{x}}}\dot\theta &{\bf{I_z}}\left(\dot\rho +\dot\psi \cos \theta \right)\end{pmatrix}=
$$

$$
  \begin{pmatrix}            \dot\theta{\bf{I_z}}(\dot\rho+\dot\psi\cos\theta)                           & - & \dot\theta{\bf{I_x}}\dot\psi\cos\theta \\        (\dot\psi  \cos \theta ) (-{\bf{I_x}} \dot\psi  \sin \theta )             & - &  ({\bf{I_z}} (\dot\rho +\dot\psi  \cos \theta) (-\dot\psi  \sin \theta)\\            -\dot\psi\sin\theta{\bf{I_x}}\dot\theta                                     & - & {\bf{I_x}}-\dot\psi\sin\theta \, \dot\theta                              \end{pmatrix}
$$

Grouping the terms group 3: 

$$
\begin{pmatrix}-{\bf{I_x}} \dot\theta \dot\psi \cos\theta +{\bf{I_z}} \dot\theta \dot\psi \cos\theta + {\bf{I_z}} \dot\theta \dot\rho \\- {\bf{I_x}} \dot\psi ^2 \cos  \theta  \sin  \theta  + {\bf{I_z}} \dot\psi ^2 \cos  \theta  \sin  \theta +{\bf{I_z}} \dot\rho  \dot\psi  \sin  \theta \\ {\bf{I_x}} \dot\theta  \dot\psi  \sin  \theta - {\bf{I_x}} \dot\theta  \dot\psi  \sin  \theta    \end{pmatrix}
$$

At this point is obvious that can simplify $$Z$$ component:

$$
\begin{pmatrix}-{\bf{I_x}} \dot\theta \dot\psi \cos\theta +{\bf{I_z}} \dot\theta \dot\psi \cos\theta + {\bf{I_z}} \dot\theta \dot\rho \\- {\bf{I_x}} \dot\psi ^2 \cos  \theta  \sin  \theta  + {\bf{I_z}} \dot\psi ^2 \cos  \theta \sin  \theta +{\bf{I_z}} \dot\rho  \dot\psi  \sin  \theta \\ 0   \end{pmatrix}
$$

This leave us with the equation:

$$
\overrightarrow{\bf {M}}{^{gyro}_{A}} = \begin{pmatrix}{\bf{I_x}}\left({-\ddot\psi\sin\theta}-{\dot\psi\dot\theta\cos\theta}\right) \\{\bf{I_x}}\ddot\theta \\ {\bf{I_z}}\left({\ddot\rho +\ddot\psi\cos\theta}-{\dot\psi\dot\theta\sin\theta}\right) \\\end{pmatrix} +                \begin{pmatrix}-{\bf{I_x}} \dot\theta \dot\psi \cos\theta +{\bf{I_z}} \dot\theta \dot\psi \cos\theta + {\bf{I_z}} \dot\theta \dot\rho \\- {\bf{I_x}} \dot\psi ^2 \cos  \theta  \sin  \theta  + {\bf{I_z}} \dot\psi ^2 \cos  \theta \sin  \theta +{\bf{I_z}} \dot\rho  \dot\psi  \sin  \theta \\ 0   \end{pmatrix}
$$

$$
\overrightarrow{\bf {M}}{^{gyro}_{A}} =  \begin{pmatrix}{\bf{I_x}} (-\dot\theta  \dot\psi  \cos  \theta -\ddot\psi  \sin  \theta ) -{\bf{I_x}} \dot\theta  \dot\psi  \cos  \theta +{\bf{I_z}} \dot\theta  \dot\psi  \cos  \theta  + {\bf{I_z}} \dot\theta  \dot\rho\\{\bf{I_x}} \ddot\theta -{\bf{I_x}} \dot\psi ^2 \cos  \theta \sin  \theta +{\bf{I_z}} \dot\psi ^2 \cos  \theta  \sin  \theta  +{\bf{I_z}} \dot\rho  \dot\psi  \sin  \theta  \\{\bf{I_z}} (\ddot\rho +\ddot\psi  \cos  \theta -\dot\theta  \dot\psi  \sin  \theta )\end{pmatrix}
$$

To further simplify the equation we substitute the $$I$$ terms across the equation:

$$
if \quad{\bf{I}} =MR^2 \\ {\bf{I_x}}=  \frac{1}{4}  MR^2 = \frac{1}{4}{\bf{I}}\qquad

 {\bf{I_z}} = \frac{1}{2}MR^2 =\frac{1}{2}{\bf{I}}

$$

$$
\overrightarrow{\bf {M}}{^{gyro}_{A}} =  \begin{pmatrix}\frac{1}{4}{\bf{I}} (-\dot\theta  \dot\psi  \cos  \theta-\ddot\psi  \sin  \theta ) -\frac{1}{4}{\bf{I}} \dot\theta  \dot\psi  \cos  \theta +\frac{1}{2}{\bf{I}} \dot\theta  \dot\psi  \cos  \theta  + \frac{1}{2}{\bf{I}} \dot\theta  \dot\rho\\\frac{1}{4}{\bf{I}} \ddot\theta -\frac{1}{4}{\bf{I}} \dot\psi ^2 \cos  \theta  \sin  \theta +\frac{1}{2}{\bf{I}} \dot\psi ^2 \cos  \theta  \sin  \theta  +\frac{1}{2}{\bf{I}} \dot\rho  \dot\psi  \sin  \theta  \\\frac{1}{2}{\bf{I}} (\ddot\rho +\ddot\psi  \cos  \theta -\dot\theta  \dot\psi  \sin  \theta )\end{pmatrix}
$$

We collect common values and simplify the equation. The final equation for the momentum of the the flywheel on our VR controller is: 

$$
\tag{9}\overrightarrow{\bf {M}}{^{gyro}_{A}} =\begin{pmatrix}\frac{1}{4} {\bf{I}} (2 \dot\theta \dot\rho -\ddot\psi \sin \theta ) \\ \frac{1}{4} {\bf{I}} (\ddot\theta +\dot\psi  \sin \theta  (2 \dot\rho +\dot\psi  \cos \theta )) \\ \frac{1}{2} {\bf{I}} (\ddot\rho +\ddot\psi  \cos \theta -\dot\theta  \dot\psi  \sin \theta )\end{pmatrix}

$$

## Torque applied on the handle

The output of equation 9 or 8 represents the momentum on frame A.

To define it relative to the handle, we will need to translate it to the required system of reference. This  can be done by applying the appropriate rotation matrix:

$$
\overrightarrow{\bf {M}}{^{gyro}_{D}} =  R_{ A \rightarrow D} \cdot \overrightarrow{\bf {M}}{^{gyro}_{A}} = R_{ C \rightarrow D} \cdot R_{ A \rightarrow C} \cdot \overrightarrow{\bf {M}}{^{gyro}_{A}}  
$$

The transformation matrix $$A \rightarrow D$$ is composed of two rotation matrices, first step $$A \rightarrow C$$ rotates over the Y-axis, and a second step $$C \rightarrow D$$ rotating over the Z-axis: 

Rotation over the Y: 

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%205.png" width="560" alt="HapticWhirl Image 5">
</p>


$$
R_{A\rightarrow C} =\begin{pmatrix}\color{red}\cos\theta & 0  & \color{blue}\sin\theta\\\color{red} 0  & 1 &\color{blue}0\\\color{red}-\sin\theta&0&\color{blue}\cos\theta\\\end{pmatrix}\\\\
$$

Rotation over the Z: 

<p align="center">
    <img src="HapticWhirl%20-%20Solving%20the%20forward%20and%20inverse%20kine%2030407d04ef9245b59da65be26bc4665f/Untitled%206.png" width="560" alt="HapticWhirl Image 6">
</p>

$$
R_{C \rightarrow D} = 
\begin{pmatrix}
\color{red}\cos\psi & \color{green}-\sin\psi  &0\\
\color{red}\sin\psi  & \color{green}\cos\psi  &0\\
\color{red}0&\color{green}0&1\\
\end{pmatrix}\\
$$

By combining both rotation matrices $$R_{A\rightarrow D} = R_{A \rightarrow C} \cdot  R_{C \rightarrow D}$$  we obtain the following rotation matri

$$
\tag{10} {R_{A \rightarrow D} =  \begin{pmatrix}\cos\theta\cos\psi & -\sin\psi  &\sin\theta\cos\psi\\\cos\theta\sin\psi  & \cos\psi  &\sin\theta\sin\psi\\-\sin\theta&0&\cos\theta\\\end{pmatrix}\\}
$$

However when we measure using the ATI sensor we got an inverted value of Z, changing the value of the z value to correct the model to match the readings. 

using:

$$
R_{A\rightarrow C} =\begin{pmatrix}\color{red}\cos\theta & 0  & \color{blue}\sin\theta\\\color{red} 0  & 1 &\color{blue}0\\\color{red}\sin\theta&0&\color{blue}\cos\theta\\\end{pmatrix}\\\\
$$

$$
\tag{10} {R_{A \rightarrow D} =  \begin{pmatrix}\cos\theta\cos\psi & -\sin\psi  &\sin\theta\cos\psi\\\cos\theta\sin\psi  & \cos\psi  &\sin\theta\sin\psi\\\sin\theta&0&\cos\theta\\\end{pmatrix}\\}
$$

Finally, the torque applied on the handle is computed combining (7) and (8) as: 

$$
\overrightarrow{\bf {M}}{^{gyro}_{D}} =  R_{ A \rightarrow D} \cdot \overrightarrow{\bf {M}}{^{gyro}_{A}}
$$

# Solving for the different moments

If we have a given torque and we want to replicate this on the controller we solve using the acceleration of the gimbal axis. To solve the for the different variables we usXe equation 9.

## Solving for $$\ddot\psi$$

Yaw acceleration is part of the $$X \text{ and } Z$$  axis. We solve first using the $$X$$component: 

$$
\overrightarrow{\bf {M}}{^{gyro}_{Ax}} = \frac{1}{4} {\bf{I}} (2 \dot\theta \dot\rho -\ddot\psi \sin \theta )
$$

$$
2\dot\theta \dot\rho - \ddot\psi\sin\theta =\frac{4\overrightarrow{\bf {M}}{^{gyro}_{Ax}}}{{{\bf{I}} }}
$$

$$
-\ddot\psi\sin\theta =\frac{4\overrightarrow{\bf {M}}{^{gyro}_{Ax}}}{{{\bf{I}} }} -2\dot\theta\dot\rho 
$$

$$
\ddot\psi = \frac{2\dot\theta\dot\rho}{\sin\theta} - \frac{4\overrightarrow{\bf {M}}{^{gyro}_{Ax}}}{{\bf{I}}\sin\theta } 
$$

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

