
#include "imm_ukf_pda.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imm_ukf_pda_tracker");
  ImmUkfPda app;
  ros::spin();

  return 0;
}
