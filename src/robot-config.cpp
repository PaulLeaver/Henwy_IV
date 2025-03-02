#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor RF = motor(PORT14, ratio6_1, false);
motor RB = motor(PORT18, ratio6_1, false);
motor LF = motor(PORT7, ratio6_1, true);
motor LB = motor(PORT8, ratio6_1, true);
inertial Inertial = inertial(PORT1);
rotation rotation1 = rotation(PORT4, false);
rotation PodF = rotation(PORT3, false);
rotation PodS = rotation(PORT5, false);
controller Controller1 = controller(primary);
motor LM = motor(PORT6, ratio6_1, false);
motor RM = motor(PORT15, ratio6_1, true);
digital_out RushMech = digital_out(Brain.ThreeWirePort.C);
digital_out Doinker = digital_out(Brain.ThreeWirePort.B);
digital_out Clamp = digital_out(Brain.ThreeWirePort.E);
motor arm = motor(PORT10, ratio18_1, false);
motor Hooks = motor(PORT11, ratio6_1, false);
motor Intake = motor(PORT20, ratio18_1, false);
optical ColorSort (PORT9);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}