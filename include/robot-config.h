using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RF;
extern motor RB;
extern motor LF;
extern motor LB;
extern inertial Inertial;
extern rotation rotation1;
extern rotation PodF;
extern rotation PodS;
extern controller Controller1;
extern motor LM;
extern motor RM;
extern digital_out RushMech;
extern digital_out Doinker;
extern digital_out Clamp;
extern motor arm;
extern motor Intake;
extern motor Hooks;
extern optical ColorSort;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );