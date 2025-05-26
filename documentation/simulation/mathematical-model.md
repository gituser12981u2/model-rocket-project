# Mathematical Model

## Current equations of translation motions (solved for acceleration)

$$\vec{a_{cm}} = \frac{\vec{dv_{cm}}}{dt} = \frac{\vec{F_t}}{m}-\frac{F_d}{m}-g-\frac{\vec{v_{cm}}}{m}\frac{dm}{dt}$$

### Components

$$a_{cm}\hat{z} = \frac{dv_{cm}\hat{z}}{dt} = \frac{F_t\hat{z}}{m}-\frac{F_d}{m}-g-\frac{v_{cm}\hat{z}}{m}\frac{dm}{dt}$$
$$a_{cm}\hat{y} = \frac{dv_{cm}\hat{y}}{dt} = \frac{F_t\hat{y}}{m}-\frac{F_d}{m}-\frac{v_{cm}\hat{y}}{m}\frac{dm}{dt}$$
$$a_{cm}\hat{x} = \frac{dv_{cm}\hat{x}}{dt} = \frac{F_t\hat{x}}{m}-\frac{F_d}{m}-\frac{v_{cm}\hat{x}}{m}\frac{dm}{dt}$$

## Account for TSD drag and general drag

### General aerodynamic drag

$$-\frac{1}{2}\rho v^2 C_D A \hat{v}$$

### TSD equation

$$(1-M^2_{\infty} - (\gamma + 1)M^2_{\infty} \frac{\partial \phi}{\partial x}) \frac{\partial^2 \phi}{\partial x^2} + \frac{\partial^2 \phi}{\partial y^2} + \frac{\partial^2 \phi}{\partial z^2} = 0$$

Assume irrotational flow of air, $\nabla \times \phi = 0$. Then
$$v_x=\frac{\partial \phi}{\partial x}$$
$$v_y=\frac{\partial \phi}{\partial y}$$
$$v_z=\frac{\partial \phi}{\partial z}$$
$$(1-M^2_{\infty} - (\gamma + 1)M^2_{\infty} v_x) \frac{\partial v_x}{\partial x} + \frac{\partial v_y}{\partial y} + \frac{\partial v_z}{\partial z} = 0$$
$$\nabla \cdot v = \frac{\partial v_x}{\partial x} + \frac{\partial v_y}{\partial y} + \frac{\partial v_z}{\partial z} $$

$$(1-M^2_{\infty} - (\gamma + 1)M^2_{\infty} v_x)\nabla \cdot v = 0$$
Then add TSD component to $F_d$

$$F_d = -\frac{1}{2}\rho v^2 C_D A \hat{v} + (1-M^2_{\infty} - (\gamma + 1)M^2_{\infty} v_x)\nabla \cdot v$$

## Equation of translational motion (accounting for TSD)

$$\vec{a_{cm}} = \frac{\vec{dv_{cm}}}{dt} = \frac{\vec{F_t}}{m}+\frac{1}{m}(-\frac{1}{2}\rho v^2 C_D A \hat{v} + (1-M^2_{\infty} - (\gamma + 1)M^2_{\infty} v_x)\nabla \cdot v)-g-\frac{\vec{v_{cm}}}{m}\frac{dm}{dt}$$
