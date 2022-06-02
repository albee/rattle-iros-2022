#include "rattle_acado_planner/rattle_acado_planner.h"

int main() {
  /* Verify ACADO is running correctly
  */
  RattlePlanner p;
  ros::Time::init();
  double tic = ros::Time::now().toSec();
  double time_elapsed = (double)acado_toc(&p.t);  // note down time elapsed to use in the exponential decay

  p.update_online_data(&p.params_model_);
  p.use_latest_info_gain();

  // at least two SQP iterations---needed for good solution!
  acado_preparationStep();   // Prepare for solving
  acado_feedbackStep();  // where the magic happens
  acado_preparationStep();   // Prepare for solving
  acado_feedbackStep();  // where the magic happens, again
  acado_preparationStep();   // Prepare for solving
  acado_feedbackStep();  // where the magic happens, again, again
  
  p.iter_++;

  double toc = ros::Time::now().toSec();
  if (VERBOSE) {
    std::cout << "Inputs: " << acadoVariables.u[0] << " " << acadoVariables.u[1] << " " << acadoVariables.u[2] << std::endl;
    std::cout << "Computation time: " << toc - tic << std::endl;
    printf("Iteration %d:  KKT Tolerance = %.3e Objective = %.3e\n", p.iter_, acado_getKKT(),
            acado_getObjective());
    printf("Info weights: %f %f %f %f\n", p.INFO_WEIGHT_M, p.INFO_WEIGHT_CX, p.INFO_WEIGHT_CY, p.INFO_WEIGHT_IZZ);
    std::cout << "Params: " << p.params_model_ << std::endl;
    p.print_data();

    // if (shift_global_setpoints) {
    //   shift_start_point(); // the goal point in the last local plan becomes the start point for the next one
    //   update_psi();
    // }
  }
}