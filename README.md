# Master-Thesis: Constrained Bayesian Optimization for Industrial Compressor Stations

## Overview

This project presents a data-driven real-time optimization framework for industrial gas compressor stations operating in serial and parallel configurations.

The goal is to:

- Minimize total power consumption

- Ensure strict surge avoidance

- Maintain precise pressure / mass-flow tracking

- Operate without relying on uncertain compressor efficiency maps

Instead of using traditional model-based load-sharing, the plant is treated as a black-box system and optimized using Safe Bayesian Optimization (Safe BO) combined with Genetic Algorithms.
This work was developed and validated in a MATLAB/Simulink simulation environment and demonstrates measurable energy savings compared to industrial equal-load strategies 


.

# 1. Industrial Motivation

Compressor stations are critical in:

- Natural gas transport

- LNG plants

- Air separation

- CO₂ and H₂ pipeline systems

Industrial practice often applies equal load distribution, which does not account for compressor efficiency differences and degradation.

Because compressor maps are uncertain and time-varying, classical two-step model-update + optimization approaches do not guarantee optimal operation under plant-model mismatch 

This project addresses that limitation using a fully data-driven, uncertainty-aware optimization framework.

# 2. System Modeling

Each compressor is described dynamically by:

- Pressure states

- Mass flow

- Pressure ratio

- Shaft speed

- Recycle dynamics

Power is computed via a polytropic thermodynamic model.

Two configurations were studied:

- Parallel Configuration, with a primary objective to track total mass flow
- Serial Configuration, with a primary objective to track discharge pressure


Decision variables: Torque allocation per compressor

Constraints:

- Actuation limits

- Flow tracking tolerance

- Distance-to-surge ≥ safety margin


# 3. Optimization Framework

**Gaussian Process (GP) Surrogates:**

Unknown steady-state mappings are learned online:

- Total Power

- Tracking signal (pressure or flow)

- Distance-to-surge for each compressor

GPs provide:

- Posterior mean

- Predictive uncertainty

==> This allows probabilistic safety guarantees.

**Constrained Acquisition Strategies**

Two strategies were implemented and compared:

- Constrained Expected Improvement (CEI): Maximizes expected power improvement weighted by Probability of surge feasibility and reference tracking weight

- UCB + Log-Barrier (UCB-LB) Minimizes a composite objective: Conservative power estimate (LCB/UCB), Quadratic tracking penalty and Log-barrier safety term for surge margins


**Genetic Algorithm (Inner Optimizer)**

Used to solve the acquisition problem at each iteration.

Key parameters:

- Population size

- Generations

- Crossover rate

- Mutation rate

These hyperparameters are tuned such as they balance exploration quality with real-time feasibility 


# 4. Simulation Environment

- MATLAB / Simulink implementation

- Steady-state detection module

- Online GP updates at steady state

- Real-time load redistribution

- Both serial and parallel case studies were implemented.

# 5. Results

Energy Savings (Serial Case)
|Method	| Energy (J)| Saving vs Baseline |
|-------|-------|------|
|Equal Load |	2.252×10⁹ |	— |
|CEI |	2.201×10⁹	|2.28% |
|UCB-LB |	2.190×10⁹|	2.76%|
|Theoretical Optimum |	2.164×10⁹ |	3.95%|


**Key Observations**

- Tracking error < 0.01% relative deviation 

- Distance-to-surge strictly positive throughout experiments

- Robust convergence across multiple random initializations

# 6. Limitations

- Steady-state safety only (transients not formally guaranteed) 

- Genetic Algorithm provides no global optimality guarantee

- No measurement noise included

- Only a few acquisition functions evaluated


# 7. Future Work

- Transient-safe BO extension

- Real-plant deployment (hybrid serial-parallel compressor networks)

- Noise-robust steady-state detection


# 8. Conclusion

This project demonstrates that a fully data-driven Constrained Bayesian Optimization framework Combined with uncertainty-aware constraint handling can reduce compressor station energy consumption While strictly preserving operational safety
It bridges industrial process control with probabilistic machine learning and real-time optimization.
