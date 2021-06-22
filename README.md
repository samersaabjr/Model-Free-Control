This repository contains implementations of the Quadruple Tank Process simulation for the model-free non-adaptive controller proposed in "Multivariable Non-adaptive Controller Design" (DOI: 10.1109/TIE.2020.2998753) in MATLAB.

This is a discrete-time robust multivariable (nonadaptive) tracking controller that comes with a simple structure, requires very limited information on the plant model, and is relatively easy to tune. In addition to being easy to tune and implement, an objective of this controller is to deal with a class of large-scale systems with complex dynamics. 

**Experiment**

The experiment considered in this repository compares the performance of the proposed controller with the controllers in [1], [2], and [3]. The former controller considers the problem of designing a multivariable PID controller via direct optimal linear quadratic regulator, and the multivariable PID gains of the latter two controllers are obtained by minimizing the error covariance matrix of different augmented systems.

The considered continuous-time system of a quadruple tank process with non-minimum phase setting is as described in [1]. The system is discretized with sampling period Ts=0.1 sec. The proposed controller parameters used in both settings (with noise and without noise) are K(k) = 1/(k^{0.2}) * 500I and gamma = 1.1. The gain is divided by by k^{0.2} as descirbed in Section III-A of the published paper so that the controller can partly reject measurement noise.

![output_response_QTP](https://user-images.githubusercontent.com/44982976/122838947-11906c80-d2c5-11eb-8e21-96b11825e020.png)

The figure above shows the output response in the presence and absence of measurement noise.

The table below shows the transient performance of the four compared controllers, where the percentage overshoot, OS%, and the settling time, ts, are shown.

Output | [1] | [2] | [3] | Proposed
--- | --- | --- | --- | ---
y1: OS% | 50% | 400 | 25% |0%
y2: OS% | 20% | 25% | 7% | 25%
y1: ts (sec) | 400s | 400s | 36s | 5s
y2: ts (sec) | 400s | 200s | 20s | 5s

The table below shows the transient performance of the proposed controller in the presence of noise, N(0, 0.05^{2}), which lists the standard deviation of the error at the time, t, during the first 100 seconds, over the entire range (2000 sec), and during the last 100 seconds.

St. Dev. | t in [0,100] sec | t in [0,2000] sec | t in [1900,2000] sec
--- | --- | --- | ---
std(y1_ref - y1) | 0.065 | 0.026 | 0.019
std(y2_ref - y2) | 0.024 | 0.030 | 0.016

**How to Execute Code**

The file "QuadrupleTank_P_type_clean.m" contains the implementation of the model-free controller on a quadruple tank process with non-minimum phase setting as described in [1]. To run the code, simply run "QuadrupleTank_P_type_clean.m".

**References**

[1] Pradhan, Jatin K., and Arun Ghosh. "Multi-input and multi-output proportional-integral-derivative controller design via linear quadratic regulator-linear matrix inequality approach." IET Control Theory & Applications 9.14 (2015): 2140-2145.

[2] Saab, Samer S. "Development of multivariable PID controller gains in presence of measurement noise." International Journal of Control 90.12 (2017): 2692-2710.

[3] Saab, Samer S. "An optimal stochastic multivariable PID controller: a direct output tracking approach." International Journal of Control 92.3 (2019): 623-641.
