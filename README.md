# Fast MPC (Model predictive control)
This repository contains matlab interface to convert a standard model predictive control to fast model predictive control based on the paper [Fast Model predictive control using online optimization].

[Fast Model predictive control using online optimization]:http://stanford.edu/~boyd/papers/pdf/fast_mpc.pdf 

## Description
In the conventional method, a MPC problem is solved and the first control step is applied to the system, and the next integrated state forms the initial condition for the next MPC iteration. Here the structure of the MPC is exploited for accelerated results. The current implementation is performed on time invariant system dynamics (equality constraints)
    http://latex.codecogs.com/gif.latex?%5Cbegin%7Beqnarray*%7D%20%5Cmin_%7Bx%28%5Ccdot%29%2Cu%28%5Ccdot%29%7D%20%26%20%26%20%5Cint%5Climits_%7B0%7D%5E%7BT%7D%20%5Cleft%28%7C%7C%20x%28t%29%7C%7C_Q%5E2%20&plus;%20%7C%7C%20u%28t%29%7C%7C_R%5E2%5Cright%29%20%5C%2C%20%5Cmathrm%7Bd%7Dt%20%5C%2C%20&plus;%20%5C%2C%20%7C%7C%20x%28T%29%20%7C%7C_P%5E2%5C%5C%20%5Ctextrm%7Bs.t.%7D%20%26%20%26%20%5Cdot%20x%28t%29%20%5C%3B%20%3D%20%5C%3B%20f%28%5C%2Cx%28t%29%2C%5C%2Cu%28t%29%5C%2C%29%2C%5C%5C%20%26%20%26%20x%280%29%20%5C%3B%20%3D%20%5C%3B%20x_0%2C%5C%5C%20%26%20%26%20%5Cunderline%20u%28t%29%20%5C%3B%20%5Cleq%20%5C%3B%20u%28t%29%20%5C%3B%20%5Cleq%20%5C%3B%20%5Coverline%20u%28t%29%2C%5C%5C%20%26%20%26%20%5Cunderline%20x%28t%29%20%5C%3B%20%5C
