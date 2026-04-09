# EV-optimization-under-Traffic-Vehicle-dynamics
This provides a framework to evaluate optimal speed profile for minimizing energy consumption based on co-simulation integrating MATLAB and VISSIM

## EV Model Update

When `get_evmodel.m` changes, update files in this order:

1. `get_evmodel.m`
   Add or change `EVout.xxx`.
2. `get_pred_hist.m`
   Add the field if it should appear in prediction logs.
3. `get_appd_hist.m`
   Add the field if it should appear in applied/base logs.
4. `get_plot.m`
   Add plotting only if you want to visualize it.

Short rule:

`get_evmodel -> get_pred_hist -> get_appd_hist -> get_plot`
