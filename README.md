This repository contains implementations of the Quadruple Tank Process simulation for the model-free non-adaptive controller proposed in "Multivariable Non-adaptive Controller Design" (DOI: 10.1109/TIE.2020.2998753) in MATLAB.

This is a discrete-time robust multivariable (nonadaptive) tracking controller that comes with a simple structure, requires very limited information on the plant model, and is relatively easy to tune. In addition to being easy to tune and implement, an objective of this controller is to deal with a class of large-scale systems with complex dynamics. 

**Experiment**

In this example we compare the performance of the proposed controller with the optimal stochastic PID controller in [1], [2], and that in [3]. The multivariable PID gains of the former two controllers are obtained by minimizing the error covariance matrix of different augmented systems, and the latter considers the problem of designing a multivariable PID controller via direct optimal linear quadratic regulator.

The considered continuous-time system of a quadruple tank process with non-minimum phase setting is as described in [1]. The system is discretized with sampling period Ts=0.1 sec. The proposed controller parameters used in both settings (with noise and without noise) are K(k) = 1/(k^{0.2}) * 500I and gamma = 1.1. The gain is divided by by k^{0.2} as descirbed in Section III-A of the published paper. so that the controller can partly reject measurement noise.

![output_response_QTP](https://user-images.githubusercontent.com/44982976/122838947-11906c80-d2c5-11eb-8e21-96b11825e020.png)

The figure above shows the output response in the presence and absence of measurement noise.

The table below shows the transient performance in presence of noise, N(0,0.05^{2}).

   | [1] | [2] | [3] | Proposed
   | --- | --- | --- | ---
y1: OS% | 50% | 400 | 25% |0%
y2: OS% | 20% | 25% | 7% | 25%
y1: ts | 400 | 400 | 36 | 5
y2: ts | 400 | 200 | 20 | 5
y1: ts (sec) | 400s | 400s | 36s | 5s
y2: ts (sec) | 400s | 200s | 20s | 5s

**How to Execute Code**

The file "QuadrupleTank_P_type_clean.m" contains the implementation of the model-free controller on a quadruple tank process with non-minimum phase setting as described in [1].

**References**

[1] Pradhan, Jatin K., and Arun Ghosh. "Multi-input and multi-output proportional-integral-derivative controller design via linear quadratic regulator-linear matrix inequality approach." IET Control Theory & Applications 9.14 (2015): 2140-2145.

[2] Saab, Samer S. "Development of multivariable PID controller gains in presence of measurement noise." International Journal of Control 90.12 (2017): 2692-2710.

[3] Saab, Samer S. "An optimal stochastic multivariable PID controller: a direct output tracking approach." International Journal of Control 92.3 (2019): 623-641.
