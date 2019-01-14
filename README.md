# py_safety_benchmark

```bash
#create a virtual environment named psb_env, using the latest python3.6
conda create -n psb_env python=3.6
#activate the virtual env
source activate psb_env
#install panda3d, this is a 3d game engine we use to render graphics
pip install --pre --extra-index-url https://archive.panda3d.org/ panda3d
pip install cvxopt
#to use matplotlib.pyplot in a conda virtual environment, we need to install python as a framework
conda install python.app
#to convert the figure to latex code, we need this lib
pip install matplotlib2tikz
```

### Potential Field

$$
U_{static} = \frac{\eta}{2} (\frac{1}{d} - \frac{1}{d_{min}})^2 \\
U_{dynamic} = \lambda(-cos \theta)^{\beta}\frac{||v_{rel}||}{d}\\
cos\theta = \frac{v_{rel}^T  x_{rel}}{||v_{rel}|| d}\\
U = U_{static}+U_{dynamic}\\
u = -\nabla_x U
$$


$$
\dot \phi = \frac{\partial \phi}{\partial X_r}^T \dot X_r + \frac{\partial \phi}{\partial X_h}^T \dot X_h -\eta -\lambda = \frac{\partial \phi}{\partial X_r}^T (fx + fu\cdot u) + \frac{\partial \phi}{\partial X_h}^T \dot X_h -\eta -\lambda\\
u = solve(\phi, \dot \phi)\\
$$




### Safe Set

$$
\phi = d_{min}^2 + \eta \cdot dt + \lambda \cdot dt - d^2 - k_v\dot d\\\
        \dot \phi \leq -\eta_i \text{ or } \phi < 0 \\
        \dot \phi = \frac{\partial \phi}{\partial X_r}^T \dot X_r + \frac{\partial \phi}{\partial X_h}^T \dot X_h -\eta -\lambda = \frac{\partial \phi}{\partial X_r}^T (fx + fu\cdot u) + \frac{\partial \phi}{\partial X_h}^T \dot X_h -\eta -\lambda\\
        
        L = \frac{\partial \phi}{\partial X_r}^T  fu\\
        S = - \eta - \lambda - \frac{\partial \phi}{\partial X_h}^T \dot X_h - \frac{\partial \phi}{\partial X_r}^T fx\\
        Lu < S\\
$$



### Barrier Function

$$
h = (d - d_{min} + \dot d  t_{react})\\

B = -log(h / (1+h))\\
\dot B = \frac{\partial B}{\partial X_r} \dot X_r + \frac{\partial B}{\partial X_h} \dot X_h = \frac{\partial B}{\partial X_r} (fx + fu\cdot u) + \frac{\partial B}{\partial X_h} \dot X_h\\
L_fB = \frac{\partial B}{\partial X_r}^T  fx\\
L_gB = \frac{\partial B}{\partial X_r}^T  fu\\
\dot B < \frac{\gamma}{B}\\

A = [L_gB]\\
b = \frac{\gamma}{B} - L_f B - \frac{\partial B}{\partial X_h} \dot X_h\\
A u < b\\
QP(A,b)\\
$$



