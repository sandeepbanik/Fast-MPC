# Fast MPC (Model predictive control)
This repository contains matlab interface to convert a standard model predictive control to fast model predictive control based on the paper [Fast Model predictive control using online optimization]. The fast MPC class solves using a custom built **infeasible start newton solver** explointing the structure of MPC. 

[Fast Model predictive control using online optimization]:http://stanford.edu/~boyd/papers/pdf/fast_mpc.pdf 

## Description
In the conventional method, a MPC problem is solved and the first control step is applied to the system, and the next integrated state forms the initial condition for the next MPC iteration. Here the structure of the MPC is exploited for accelerated results. The current implementation is performed on time invariant system dynamics (equality constraints)

![](http://latex.codecogs.com/gif.latex?%5Cbegin%7Beqnarray*%7D%20%5Cmin_%7Bx%28%5Ccdot%29%2Cu%28%5Ccdot%29%7D%20%26%20%26%20l_%7Bf%7D%28x%28t&plus;T%29%29%20&plus;%20%5Csum_%7Bn%3D%5Ctau%7D%5E%7B%5Ctau&plus;T-1%7D%20l%28%20x%28%5Ctau%29%2Cu%28%5Ctau%29%20%5C%2C%5C%5C%20%5Ctextrm%7Bs.t.%7D%20%26%20%26%20x%28t&plus;1%29%20%5C%3B%20%3D%20%5C%3B%20A%28%5C%2Cx%28t%29%29&plus;B%28%5C%2Cu%28t%29%5C%2C%29%20&plus;%20%5Chat%7Bw%7D%5C%5C%20%26%20%26%20x%280%29%20%5C%3B%20%3D%20%5C%3B%20x_0%2C%5C%5C%20%26%20%26%20%5Cunderline%20u%28t%29%20%5C%3B%20%5Cleq%20%5C%3B%20u%28t%29%20%5C%3B%20%5Cleq%20%5C%3B%20%5Coverline%20u%28t%29%2C%5C%5C%20%26%20%26%20%5Cunderline%20x%28t%29%20%5C%3B%20%5Cleq%20%5C%3B%20x%28t%29%20%5C%3B%20%5Cleq%20%5C%3B%20%5Coverline%20x%28t%29%2C%20%5Cquad%20%5Ctextrm%7Bfor%20all%7D%20%5C%3B%20%5C%2C%20t%20%5Cin%20%5B0%2CT%5D%5C%3B%20%5Cend%7Beqnarray*%7D)

Where,

![](http://latex.codecogs.com/gif.latex?%24%24%20l_%7Bf%7D%28x%28t&plus;T%29%29%20%3D%20x%28t&plus;T%29%5E%5Cintercal%20Q_%7Bf%7Dx%28t&plus;T%29%20&plus;%20q_%7Bf%7D%5E%5Cintercal%20x%28t&plus;T%29%20%24%24)
![](http://latex.codecogs.com/gif.latex?l%28x%28%5Ctau%29%2Cu%28%5Ctau%29%29%20%3D%20%5Cbegin%7Bbmatrix%7Dx%28t%29%20%26%20u%28t%29%20%5Cend%7Bbmatrix%7D%5Cbegin%7Bbmatrix%7DQ%20%26%20S%5E%5Cintercal%20%5C%5C%20S%20%26%20R%20%5Cend%7Bbmatrix%7D%5Cbegin%7Bbmatrix%7Dx%28t%29%20%5C%5C%20u%28t%29%20%5Cend%7Bbmatrix%7D%20&plus;%20q%5E%5Cintercal%20x%28t%29%20&plus;%20r%5E%5Cintercal%20u%28t%29)

The above mentioned problem is converted into the following structure


![](http://latex.codecogs.com/gif.latex?minimize%20%5Cquad%20z%5E%5Cintercal%20Hz%20&plus;%20g%5E%5Cintercal%20z%20&plus;%20k%5Cphi%28z%29)
![](http://latex.codecogs.com/gif.latex?subject%20%5C%20to%20%5Cquad%20Cz%20%3D%20b)

where,
![](http://latex.codecogs.com/gif.latex?%5Cphi%28z%29%20%3D%20%5Csum_%7Bi%3D1%7D%5E%7BlT&plus;k%7D%20-log%28h_%7Bi%7D%20-%20p_%7Bi%7D%5E%5Cintercal%20z%29)

The details of the matrices and structure are present in the paper. 

**Upcoming updates will include non-linear fast MPC along with the inclusions of integrators**


## Getting Started

Clone or download the repository. Various inputs are needed to the FAST MPC class which are explained as follows.

Q - State stage cost

R - Control stage cost

S - State control coupled cost
Qf - Terminal state cost
q - linear state cost
r - linear control cost
qf - terminal state linear cost
x_min - lower bound on state
x_max - upper bound on state
u_min - lower bound on control
u_max - upper bound on control
T - Horizon length
x0 - initial state
A - State transition matrix
B - Control matrix
w - distrubance vector
xf - final state
x_init - initialization(optional) (Note: if you intend to provide your own initialization, then the entire vector length needs to be provided i.e., T*(n+m))

A constructor is also present for the class and the cronological order is as mentioned above.

### Prerequisites

Matlab 2010 or higher


## Running the tests

The test_fast_mpc.m contains a random system. It also compares the native matlab solver and various fast MPC methods.


## Authors

* **Sandeep Banik** -  [Projects](https://github.com/sandeepbanik)

## Reference 

**Fast Model Predictive Control Using Online Optimization** by Yang Wang and Stephen Boyd

