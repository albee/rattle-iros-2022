# inv_fam
A ROS package for the [Astrobee Robot Software](https://github.com/nasa/astrobee) to obtain applied (post-saturation) forces and torques from nozzle positions. Relevant for model-based estimation approaches.

## Usage
To be used with the Astrobee simulation software. Either launch independently as `roslaunch inv_fam inv_fam.launch`, or add to to Astrobee's MLP.launch to launch with the other gnc nodelets.

## Notes
The inv_fam module performs an inversion of Astrobee's Force Allocation Module, i.e., computes the forces and torques from commanded nozzle positions.

subscriptions: `/hw/pmc/command` for nozzle positions and /mob/inertia for getting the current center of mass offset for the mixer
publications: `/inv_fam/appliedFandT`

If `diagnostics = 1`, the FAM is replicated, with 
additional subscriptions: `/gnc/ctl/command` for commanded F&T and `/mob/inertia`
publications: `fam/commandedNozzles`







