# ASAP ROS and manager components
add_subdirectory(rattle_msgs)
add_subdirectory(coordinator)
add_subdirectory(rattle_coordinator)
add_subdirectory(execute_asap)
add_subdirectory(data)

# Global planning
add_subdirectory(rattle_rrt)

# Local information-aware planning
add_subdirectory(rattle_acado_planner)

# RATTLE control
add_subdirectory(casadi_nmpc)
add_subdirectory(z_poly_calc)

# RATTLE online parameter estimation
add_subdirectory(param_est)
add_subdirectory(inv_fam)
