# ASAP
add_subdirectory(reswarm_msgs)
add_subdirectory(coordinator)
add_subdirectory(execute_asap)
add_subdirectory(data)

# Smoothing LQR-RRT*
add_subdirectory(planner_lqrrrt)

# RATTLE planning and coordianation
add_subdirectory(rattle_rrt)
add_subdirectory(rattle_acado_planner)
add_subdirectory(rattle_coordinator)

# RATTLE control
add_subdirectory(casadi_nmpc)
add_subdirectory(z_poly_calc)

# RATTLE online parameter estimation
add_subdirectory(param_est)
add_subdirectory(inv_fam)

# RATTLE pre-generated info traj
add_subdirectory(info_rich_traj)

# DMPC
add_subdirectory(reswarm_dmpc)

