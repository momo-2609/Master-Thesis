# Constrained Bayesian Optimization for Safe Load-Sharing Management of Compressor Stations

**Degree:** Master Thesis — ETH Zurich  
**Institute:** Institute for Dynamic Systems and Control (IDSC)  
**Author:** Mohamed Becha  
**Supervisors:** Prof. Dr. Melanie Zeilinger, Prof. Dr. Johannes Köhler, Dr. Mehmet Mercangöz  
**Date:** March 2026

> Parts of this thesis have been prepared for submission to the *Applied Energy Journal* under the title "Real-Time Constrained Bayesian Optimization for Safety-Aware Load-Sharing in Compressor Stations" (Dong, Becha, Köhler, Zagorowska, Mercangöz).

---

## Overview

Industrial gas compressor stations — used in pipeline transport, LNG production, CO₂/H₂ infrastructure, and air separation — typically operate multiple compressors arranged in parallel or in series. In current industrial practice, load is distributed equally across units, which ignores differences in individual compressor efficiencies and leads to suboptimal energy consumption.

This thesis develops a **data-driven, model-free supervisory framework** that optimally redistributes shaft torques across compressors to minimize total power consumption, while simultaneously tracking the plant demand setpoint and ensuring that each compressor remains safely to the right of its surge control line.

The key idea is to treat the compressor station as a **black-box system**: instead of relying on explicit compressor performance maps (which degrade over time due to fouling, wear, and erosion), the framework learns the input-output mapping from measured data using Gaussian Process surrogate models, and uses Constrained Bayesian Optimization (CBO) to iteratively improve the operating point.

---

## Problem Statement

### Load-Sharing Optimization (LSO)

The goal is to find shaft torque allocations **T** = [T₁, T₂, ..., Tₙ] that solve:

```
minimize   P(T)              [total power consumption]
subject to h(T) = h_ref      [plant demand tracking]
           Sᵢ(T) ≥ m         [surge safety for each compressor i]
           T ∈ [T_min, T_max] [actuator bounds]
```

where:
- `P(T)` is the total power consumed by all compressors
- `h(T)` is the demand variable — total mass flowrate for parallel, discharge pressure for serial
- `Sᵢ(T)` is the surge control distance of compressor *i* (must stay positive)
- `m ≥ 0` is a prescribed safety margin

### Why This Is Hard

Traditional LSO relies on accurate compressor maps, but these shift over time due to degradation. The two-step approach (re-identify map parameters, then optimize) cannot guarantee plant optimality due to structural model mismatch. This work bypasses the need for any compressor model whatsoever.

---

## Background

### Compressor Physics

Each compressor is modeled by a 6-state nonlinear ODE governing suction/discharge pressures, mass flow, pressure ratio, shaft speed, and recycle flow. The shaft power is:

```
P = (W_p / η) · m_comp
```

where W_p is the polytropic head and η the polytropic efficiency. The **surge control distance** is defined as:

```
SCD(m_comp, Π) = m_comp − (Π − s₀) / s₁
```

with s₀, s₁ defining the surge control line. Keeping SCD > 0 is the safety-critical constraint.

**Parallel configuration:** compressors share common suction/discharge headers; the demand is total outlet mass flowrate.

**Serial configuration:** discharge of compressor *i* feeds into suction of compressor *i+1*; the demand is the discharge pressure of the last stage. This introduces floating inter-stage pressure dynamics and stronger control coupling between units.

### Bayesian Optimization

Bayesian Optimization (BO) finds the optimum of an expensive black-box function f(x) using a probabilistic surrogate. At each iteration it: (1) fits a Gaussian Process (GP) to observed data, (2) maximizes an acquisition function to select the next evaluation point, (3) queries the plant, and (4) updates the model.

Two acquisition functions are used in this work:

**Constrained Expected Improvement (CEI):**
```
CEI(T) = EI(T) · ∏ᵢ PoFᵢ(T)
```
where EI is the expected improvement on power and PoF is the joint probability of feasibility for surge constraints. Exploratory and risk-aware.

**Log-Barrier (LCB-LB):**
```
F(T) = w_P · (μ_P − κ_P σ_P) + w_track · (μ_h − h_ref)² − w_surge · Σᵢ log(LCB_Sᵢ − m)
```
Interior-point mechanism: the log term goes to +∞ as the predicted surge margin approaches the boundary. More conservative and stable.

Both are paired with **probabilistic safety guarantees**: if the lower confidence bound on a surge constraint exceeds the safety margin m, the true constraint is satisfied with probability ≥ 1−δ (frequentist, uniform over the domain).

### Genetic Algorithm (GA)

The inner optimization of the acquisition function is non-convex, so a real-valued GA is used as a derivative-free global optimizer. The GA evolves a population of torque candidates through selection, crossover, and mutation. Elitism ensures the best solution is never lost.

---

## Framework

### Black-Box Surrogate Modeling

The compressor station is modeled as a steady-state mapping **y = f(T)** from shaft torques to outputs. Three independent GPs are maintained:

- **P(T)**: total power — zero-mean GP, output standardized
- **h(T)**: demand variable — linear trend (OLS) + zero-mean GP on residuals, to improve extrapolation in sparse data regimes
- **Sᵢ(T)**: surge control distance for each compressor — fixed positive prior mean (m₀ = 0.07 kg/s) to enforce conservative extrapolation in unobserved regions

All GPs use an SE-ARD kernel with fixed hyperparameters. Inputs are normalized to [0,1].

### Real-Time Iterative Loop

```
Algorithm: CBO for Real-Time Load-Sharing Optimization

Input: nominal safe torque T_nom, bounds T, initial set T_INIT, novelty radius r_nov
Initialize: empty dataset D, GP priors, apply T_0 = T_nom

for each time step k:
    measure P_k, h_k, SCD_k, T_k
    if steady-state detected:
        if T_k is novel (distance > r_nov from all existing samples):
            append (T_k, P_k, h_k, SCD_k) to D
        if |D| < N_min:
            use next initialization point
        else:
            retrain GP surrogates
            construct acquisition function F(T)
            solve T_{k+1} = argmin F(T) using Genetic Algorithm
    apply T_{k+1}
```

**Novelty criterion:** a new sample is only added to the dataset if its Euclidean distance from all existing samples exceeds r_nov. This bounds the dataset size to a finite maximum M (proven via a packing argument), keeping GP retraining tractable.

**Steady-state detection:** a 20-sample sliding window checks for (1) limited amplitude variation, (2) low dispersion (median absolute deviation), and (3) negligible linear trend. Confirmed over 5 consecutive steps with hysteresis to avoid chattering.

---

## Experimental Setup

All simulations use first-principles compressor models in MATLAB/Simulink, previously validated against industrial data.

### Parallel Configuration (3 compressors)
- Objective: track total mass flowrate setpoint
- Demand schedule: 1.6 → 1.4 → 1.25 → 1.8 → 2.0 → 2.1 kg/s over 30,000 s
- Torque bounds: [9, 21] Nm per compressor
- Safety margin: m = 0.005 kg/s

### Serial Configuration (2 compressors with intercooler)
- Objective: track discharge pressure setpoint
- Demand schedule: 248 → 244 → 246 → 243 → 242 → 241 kPa over 30,000 s
- Torque bounds: [11, 21] Nm per compressor
- Safety margin: m = 0.01 kg/s (higher due to stronger inter-stage coupling)

### Evaluation Protocol
Each fitness function is tested with 5 different initialization seeds → 20 simulations per configuration. Performance is assessed on three axes: demand tracking accuracy, surge safety maintenance, and total energy savings vs. the equal-load baseline.

---

## Results

### Parallel Configuration

Both fitness functions converge rapidly to the mass flowrate reference after each step change, with median tracking error near zero.

**Energy savings vs. equal-load baseline:**

| Method | Total Energy (GJ) | Savings (%) |
|--------|------------------|-------------|
| Equal-load (baseline) | 5.904 | — |
| FF1 (CEI), best seed | 5.366 | **2.90%** |
| FF1 (CEI), average | ~5.41 | ~2.58% |
| FF2 (Log-barrier), best seed | 5.377 | 2.70% |
| FF2 (Log-barrier), average | ~5.42 | ~2.32% |
| Theoretical optimum | 5.358 | **3.05%** |

Safety: all surge control distances remain strictly above the minimum margin throughout all 10 runs, including during transient setpoint changes.

### Serial Configuration

Both fitness functions converge to the discharge pressure reference. FF2 exhibits slightly lower tracking dispersion across seeds.

**Energy savings vs. equal-load baseline:**

| Method | Total Energy (GJ) | Savings (%) |
|--------|------------------|-------------|
| Equal-load (baseline) | 3.364 | — |
| FF1 (CEI), best seed | 3.271 | **2.76%** |
| FF1 (CEI), average | ~3.30 | ~1.64% |
| FF2 (Log-barrier), best seed | 3.269 | **2.80%** |
| FF2 (Log-barrier), average | ~3.29 | ~2.29% |
| Theoretical optimum | 3.253 | **3.29%** |

Safety: surge control distances remain strictly positive throughout all 10 runs.

### Key Physical Insights

**Parallel:** The optimal torque distribution is demand-dependent. At low flows (~1.25 kg/s), compressor C1 carries a larger share while C2 and C3 operate near their surge margin. At high flows (>1.8 kg/s), C2 becomes dominant as it is the most efficient unit. The equal-load strategy misses these efficiency differences entirely.

**Serial:** The optimizer consistently assigns higher torque (higher pressure ratio) to the downstream compressor C2, which is more efficient across the full pressure range. Energy savings are largest at low discharge pressures, where the feasible operating region — satisfying both stage surge constraints simultaneously — narrows to a thin band near the diagonal of the torque space. The two compressors exhibit "antagonistic" surge behavior: pulling one away from surge tends to push the other closer.

---

## Comparison of the Two Fitness Functions

| Aspect | FF1 (CEI) | FF2 (Log-Barrier) |
|--------|-----------|------------------|
| Exploration | More exploratory | More targeted |
| Safety margin | Operates closer to constraint boundary | Maintains larger margin |
| Energy savings | Slightly higher on average (parallel) | More consistent across seeds |
| Hyperparameter tuning | No weight tuning required | Requires tuning of w_P, w_track, w_surge |
| Samples collected (parallel avg.) | 65.4 | 56.2 |
| Tracking dispersion | Slightly higher | Slightly lower |

**Overall trade-off:** CEI enables more aggressive exploration and can find better optima, but with more variability. The log-barrier approach is more conservative, predictable, and easier to tune for serial configurations where constraint coupling is tighter.

---

## Computational Performance

Per-iteration cost of the Bayesian optimization block:

| Configuration | BO calls | Cost/call |
|--------------|----------|-----------|
| Parallel (3 compressors) | 1,206 | **9.7 ms** |
| Serial (2 compressors) | 1,206 | **8.1 ms** |

Both are well within the 1s control loop sample time, confirming real-time feasibility. The novelty criterion keeps datasets compact (well below theoretical maximums of 224,693 and 2,141 points for parallel and serial respectively), bounding GP retraining cost throughout operation.

---

## Practical Considerations

The torque-based formulation maps naturally onto **variable-frequency electric drive systems**, where torque commands are directly available. For turbine-driven stations, the framework generalizes by reformulating the decision variables as fuel/steam valve positions.

For robustness to process disturbances, the CBO supervisory layer can be paired with a fast inner PID loop that rejects transient deviations between optimization cycles — a standard hierarchical control architecture.

---

## Limitations and Future Work

- **Transient safety:** the CBO framework optimizes steady-state safety only; surge constraints during torque step changes are not formally guaranteed (though empirically maintained in simulations)
- **Genetic Algorithm:** heuristic convergence with no global optimality guarantee; sensitive to hyperparameter choices
- **Noiseless simulations:** real-world measurement noise may affect steady-state detection and GP training quality
- **Fixed valve openings:** the current formulation treats valve positions as constants; incorporating them as additional degrees of freedom could yield further savings
- **Scalability:** GP complexity scales as O(N³); extension to larger compressor networks would require sparse GP approximations or alternative surrogates
- **Hybrid configurations:** real industrial stations often combine parallel and serial arrangements; extending the unified formulation to hybrid topologies is a natural next step

---

## Tools & Implementation

- **Simulation environment:** MATLAB 2025b / Simulink
- **Compressor models:** first-principles nonlinear ODEs, validated against industrial data
- **GPs:** implemented from scratch in MATLAB (SE-ARD kernel, Cholesky factorization)
- **Genetic Algorithm:** implemented from scratch in MATLAB (population: 25, generations: 20, crossover rate: 0.6, mutation rate: 0.4)
- **Theoretical optimum:** computed via exhaustive grid search in Python

**Keywords:** Compressors, Surge, Constrained Bayesian Optimization, Gaussian Processes, Genetic Algorithm, Load-Sharing Optimization, Real-Time Optimization
