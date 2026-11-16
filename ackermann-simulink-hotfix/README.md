# Ackermann Simulink — hotfix v1.0.1

This folder contains the MATLAB/Simulink source hotfix for the lecture-style **Ackermann_GiaoTrinh_DWA_TEB_NE** project (DWA / simplified TEB).

## Fixed problems

1. `TKQD_cho_P/Phuong_trinh`: MATLAB Coder reported **Variable 'q' is not fully defined on some execution paths**. Initialize `q` and `th` before the loop.
2. `msfcn_planner_giaotrinh.m`: the planner Dwork initialization must be a **(5+2*N)-by-1** vector, not a concatenation of column vectors and an N-by-2 matrix.
3. `build_ackermann_giaotrinh.m`: the model update/compile error must propagate, rather than being swallowed as a warning.

## Applying to the source package

Replace `function_scripts/tkqd_p.txt` and `msfcn_planner_giaotrinh.m` with the files in this folder. In the original builder, replace the `try/catch` surrounding `set_param(mdl,'SimulationCommand','update')` with a direct call:

```matlab
set_param(mdl,'SimulationCommand','update');
```

Then, inside MATLAB's **Current Folder** at the extracted project root, run:

```matlab
clear functions
rehash
build_ackermann_giaotrinh
```

The builder embeds the content of `function_scripts/tkqd_p.txt` into the Simulink MATLAB Function block, so rebuilding is needed.

## Validation and limits

Regression checks on the source files passed. The `.slx` file has **not** been compiled or run in MATLAB in the author's environment. If another diagnostic occurs, send the full MATLAB Diagnostic Viewer text.

The complete updated project ZIP was also delivered in the ChatGPT conversation on 2026-11-16. This GitHub directory is a versioned copy of the hotfix source files; it is not a replacement for the full package.

## Model notes

Five-state Ackermann non-slip Newton–Euler reduced model, one total rear-axle torque command, and one steering command. DWA is a sampled planner; TEB is a simplified time-elastic-band implementation with a DWA fallback, **not** the full Nav2 or Navigation Toolbox implementation.
