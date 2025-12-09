# PECRealizations

# README: Code and Data for PEC Realizations

This repository accompanies the Automatica submission on **Plant Equivalent Controller Realizations for Attack-Resilient Cyber-Physical Systems**, with the use-case **quadruple-tank process**. It provides all scripts, models, and implementation details required to reproduce the results, figures, and case studies presented in the manuscript.

---

## 1. Overview

The repository contains MATLAB and Simulink files used to:

* Solve LMI-based optimization problem for optimal PEC realization
* Conduct simulation-based evaluation, implementing the quadruple-tank case study with a decentralized PI-controller.

All optimization problems are formulated using **YALMIP**, with **MOSEK** as the LMI solver (alternatives can be used, such as LMILAB and SDPT3).

---


## 2. Requirements

### MATLAB

* MATLAB R2022b or later
* Control System Toolbox
* Optimization Toolbox

### YALMIP

Download from: [https://yalmip.github.io/download/](https://yalmip.github.io/download/)

### MOSEK

Required for reliable LMI solving.
