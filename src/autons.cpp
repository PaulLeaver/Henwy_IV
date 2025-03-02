#include "vex.h"
#include <iostream>

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 2.75, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 8;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .3;
  chassis.drive_min_voltage = 0;

  // chassis.heading_max_voltage = 8;
  // chassis.drive_max_voltage = 7;
  // chassis.drive_settle_error = 1.5;
  // chassis.drive_settle_time = 300;
  // chassis.drive_timeout = 2000;
  // chassis.boomerang_lead = .5;
  // chassis.drive_min_voltage = 0;
}

bool allow_breaking = false; // set to true to have the robot pause at every step
void log(){ // logs information about the robot
  std::cout << // function that logs stuff
  "x: " << chassis.get_X_position() // x position
  << " y: " << chassis.get_Y_position() // y position
  << " angle: " << chassis.get_absolute_heading() // direction
  << "\nintake effic: " << Intake.efficiency() // intake efficiency
  << " volt:" << Intake.voltage() // intake voltage
  << "\nTime:" << Brain.Timer.value() // Time that has gone by since start
  << std::endl; // ends the line
  if (allow_breaking) { // allows the robot to break its process while driving
    char input; // defines the input for the bot
    std::cout << "press any key to continue" << std::endl;
    std::cin >> input; // takes input
  }
}

void intake_rev_if_stuck() { // function to reverse the intake motor if it is caught on something
  bool intakeReversing = false;
  int revDelayFrames = 10;
  int fwdDelayFrames = -1;
  static directionType oldDir = forward;
  auto driveDir{[](directionType dir)-> const char* {
    if (dir == directionType::fwd) {
      return "forward";
    };
    return "reverse";
  }};
  while (1) {
    if (round(Intake.efficiency(pct)) == 0 && abs(Intake.voltage(volt)) > 4) { // checks if the motor is trying to move, but can't 
      revDelayFrames--;
      if (!intakeReversing && revDelayFrames < 0) {
        fwdDelayFrames = 10;
        intakeReversing = true; 
        if (oldDir == forward) { // ensures that the intake will always reverse from it's stuck position by rotating the exact opposite direction
          //Intake.spin(reverse);
        }
        else {
          //Intake.spin(forward);
        }
      }
    }
    else {
      fwdDelayFrames--;
      if (fwdDelayFrames < 0) {
        if (Intake.direction() != oldDir) { // make the intake start turning it's original direction once it has been unstuck
          //Intake.spin(oldDir);
        }
        revDelayFrames = 10;
        intakeReversing = false;
        oldDir = Intake.direction();
      }
    }
    std::cout << "dir: " << driveDir(Intake.direction()) << " oldDir: " << driveDir(oldDir) << "\nfwdTime: " << fwdDelayFrames << " revTime: " << revDelayFrames << "\n";
    task::sleep(50);
  }
}

void hooks_rev() {
  while (1) {
    if (abs(Hooks.efficiency()) <= .5 && abs(Hooks.voltage()) >= 5) {
      Hooks.setReversed(true);
    } 
    else {
      Hooks.setReversed(false);
    }
    this_thread::sleep_for(50);
  }
}

int BIntakeTask(){
  uint32_t lastCheckTime = vex::timer::system();
  ColorSort.integrationTime(50);
  ColorSort.setLightPower(100);
  while(true){
    Hooks.setVelocity(-100,percent);
    Intake.setVelocity(-100,percent);
    int hue = ColorSort.hue();
    if(((hue >= 200 && hue <= 260))){
      Brain.Screen.print("Blue detected! Spinning hooks.");
      Brain.Screen.newLine();
      Hooks.setVelocity(100,percent);
      Intake.setVelocity(100,percent);
      vex::this_thread::sleep_for(150);
      Hooks.setVelocity(-100,percent);
      Intake.setVelocity(-100,percent);
    }
    vex::this_thread::sleep_for(50);
  }
}

int RIntakeTask(){
  uint32_t lastCheckTime = vex::timer::system();
  ColorSort.setLightPower(100);
   ColorSort.integrationTime(50);
  while(true){
    Hooks.setVelocity(-100,percent);
    Intake.setVelocity(-100,percent);

    int hue = ColorSort.hue();
    if(((hue >= 0 && hue <= 40))){
      Hooks.setVelocity(100,percent);
      Intake.setVelocity(100,percent);
      wait(150, msec);
      Hooks.setVelocity(-100,percent);
      Intake.setVelocity(-100,percent);
    }
    wait(50, msec);
  }
}

void Blue_Solo_WP(){
    chassis.set_coordinates(0, 0, 0);
 chassis.set_drive_exit_conditions(1.5, 300, 650);
  chassis.set_drive_constants(8, 1.5, 0, 10, 0);
  Intake.setVelocity(100, percent);
  Hooks.setVelocity(100,percent);
  Clamp.set(false);
  //Driving to the Mobile goal
  chassis.drive_distance(-29);
  Clamp.set(true);
  chassis.set_turn_constants(7, .4, .03, 5, 15);
  chassis.turn_to_angle(70);
  Intake.spin(reverse);
  Hooks.spin(reverse);
  chassis.set_drive_exit_conditions(1.5, 300, 3000);
 chassis.drive_distance(25);
  chassis.turn_to_angle(-90);
  chassis.drive_timeout = 6000;
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(53);;



  // chassis.set_coordinates(0,0,0);
  // arm.setVelocity(80,percent);
  // Hooks.setVelocity(100,percent);
  // Intake.setVelocity(100, percent);
  // Doinker.set(true);
  // chassis.drive_distance(-8);
  // Doinker.set(false);
  // Intake.spin(reverse);
  // chassis.turn_to_angle(-20);
  // chassis.drive_distance(17);
  // Intake.stop();
  // chassis.drive_distance(-5);
  // chassis.turn_to_angle(10);
  // chassis.drive_distance(20);
  // chassis.drive_distance(-5);
  // chassis.set_turn_constants(12, .4, .03, 3, 15);
  // chassis.turn_to_angle(90);
  // chassis.set_drive_constants(8, 1.5, 0, 10, 0);
  // chassis.drive_distance(15);
  // arm.spinFor(forward, .75, sec);
  // arm.spinFor(reverse, .75, sec);
  // chassis.drive_distance(-6);
  // //chassis.drive_to_point(30,-14);
  // chassis.turn_to_angle(55);
  // chassis.set_drive_exit_conditions(1.5, 300, 1000);
  // chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  // chassis.drive_distance(-40);
  // Clamp.set(true);
  // chassis.set_turn_constants(6, .4, .03, 3, 15);
  // chassis.turn_to_angle(-190);
  // Intake.spin(reverse);
  // Hooks.spin(reverse);
  // chassis.drive_distance(22);
  // log();
  // chassis.drive_max_voltage = 6;
  // task::sleep(1000);
  // chassis.turn_to_angle(0);
  // // arm.spinFor(forward, .6, sec);
  // chassis.drive_distance(41);
  
}
void Red_Solo_WP(){
//   chassis.set_coordinates(0, 0, 0);
//  chassis.set_drive_exit_conditions(1.5, 300, 650);
//   chassis.set_drive_constants(8, 1.5, 0, 10, 0);
//   Intake.setVelocity(100, percent);
//   Hooks.setVelocity(100,percent);
//   Clamp.set(false);
//   //Driving to the Mobile goal
//   chassis.drive_distance(-29);
//   Clamp.set(true);
//   chassis.set_turn_constants(7, .4, .03, 5, 15);
//   chassis.turn_to_angle(-70);
//   Intake.spin(reverse);
//   Hooks.spin(reverse);
//   chassis.set_drive_exit_conditions(1.5, 300, 3000);
//   chassis.drive_distance(25);
//   chassis.turn_to_angle(90);
//   chassis.drive_timeout = 6000;
//   chassis.drive_max_voltage = 6;
//   chassis.drive_distance(53);



  chassis.set_coordinates(0,0,0);
  arm.setVelocity(80,percent);
  Hooks.setVelocity(100,percent);
  Intake.setVelocity(100, percent);
  chassis.drive_distance(13.5);
  chassis.drive_distance(-3);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.turn_to_angle(-90);
  chassis.set_drive_constants(8, 1.5, 0, 10, 0);
  chassis.drive_distance(10);
  arm.spinFor(forward, .75, sec);
  arm.spinFor(reverse, .75, sec);
  chassis.drive_distance(-6);
  //chassis.drive_to_point(30,-14);
  chassis.turn_to_angle(-55);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  chassis.drive_distance(-40);
  Clamp.set(true);
  chassis.set_turn_constants(6, .4, .03, 3, 15);
  chassis.turn_to_angle(190);
  Intake.spin(reverse);
  Hooks.spin(reverse);
  chassis.drive_distance(24);
  log();
  chassis.drive_max_voltage = 6;
  task::sleep(1000);
  chassis.turn_to_angle(-5);
  chassis.drive_distance(40);

  // wait(.5,seconds);
  // chassis.turn_to_angle(65);
  // // chassis.drive_distance(19);
  // // wait(.5,seconds);
  // // chassis.drive_distance(-19);
  // // chassis.turn_to_angle(90);
  // // chassis.drive_distance(19);
  // // wait(.5,seconds);
  // // chassis.drive_distance(-19);
  // // chassis.turn_to_angle(180);
  // // chassis.set_drive_constants(7, 1.5, 0, 10, 0);
  // // chassis.drive_distance(70);
}

void Blue_4_Ring(){
  //thread hooksRev(hooks_rev);
  chassis.set_coordinates(0, 0, 0);
  chassis.set_drive_exit_conditions(1.5, 300, 650);
  chassis.set_drive_constants(8, 1.5, 0, 10, 0);
  Intake.setVelocity(100, percent);
  Hooks.setVelocity(100,percent);
  Clamp.set(false);
  //Driving to the Mobile goal
  chassis.drive_distance(-29);
  Clamp.set(true);
  chassis.set_turn_constants(8, .4, .03, 5, 15);
  chassis.turn_to_angle(-120);
  Intake.spin(reverse);
  Hooks.spin(reverse);
  // vex::task intakeTask(Rintaketask);
  chassis.drive_distance(16);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(16);
  chassis.turn_to_angle(15);
  chassis.drive_distance(16);
  log();
  wait(.5,seconds);
  // Intake.spin(forward);
  // chassis.turn_to_angle(80);
  // chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  // chassis.set_drive_exit_conditions(1.5, 300, 1000);
  // chassis.drive_distance(20);
  // chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  // chassis.drive_distance(20);
  // log();
   
}
void Red_4_Ring(){
  chassis.set_coordinates(0, 0, 0);
  chassis.set_drive_exit_conditions(1.5, 300, 650);
  chassis.set_drive_constants(8, 1.5, 0, 10, 0);
  Intake.setVelocity(100, percent);
  Hooks.setVelocity(100,percent);
  Clamp.set(false);
  //Driving to the Mobile goal
  chassis.drive_distance(-29);
  Clamp.set(true);
  chassis.set_turn_constants(8, .4, .03, 5, 15);
  chassis.turn_to_angle(120);
  Intake.spin(reverse);
  Hooks.spin(reverse);
  // vex::task intakeTask(Bintaketask);
  chassis.drive_distance(16);
  chassis.turn_to_angle(90);
  chassis.drive_distance(16);
  chassis.turn_to_angle(-15);
  chassis.drive_distance(16);
  log();
  wait(.5,seconds);
  Intake.spin(forward);
  // chassis.turn_to_angle(-80);
  // chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  // chassis.set_drive_exit_conditions(1.5, 300, 1000);
  // chassis.drive_distance(20);
  // chassis.turn_to_angle(180);
  // chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  // chassis.drive_distance(20);
  // log();
   
  

  
}

void Red_Goal_Rush(){
  odom_constants();
  chassis.set_coordinates(0,0,0);

  Intake.setVelocity(100, percent);
  Hooks.setVelocity(100, percent);
  chassis.drive_max_voltage = 10;
  chassis.set_drive_exit_conditions(1.5, 300, 2500);
  Intake.spin(reverse);
  chassis.drive_distance(39);
  Doinker.set(true);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(-15);
  // Doinker.set(false);
  // chassis.drive_distance(-5);
  // chassis.turn_to_angle(180);
  // chassis.drive_distance(-10);
  // Clamp.set(true);
  // Intake.spin(reverse);
  // Hooks.spinFor(reverse,2,turns);
  // Clamp.set(false);
  // chassis.turn_to_angle(75);
  // chassis.drive_distance(-25);
  // Clamp.set(true);
  // Hooks.spin(reverse);
}
void Blue_Goal_Rush(){
 odom_constants();
  chassis.set_coordinates(0,0,0);

  Intake.setVelocity(100, percent);
  Hooks.setVelocity(100, percent);
  chassis.drive_max_voltage = 12;
  chassis.set_drive_exit_conditions(1.5, 300, 2500);
  Intake.spin(reverse);
  chassis.drive_distance(39);
  Doinker.set(true);
  wait(.25,seconds);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(-15);
  // chassis.drive_distance(-5);
  chassis.turn_to_angle(-10);
  chassis.drive_distance(-10);
  Clamp.set(true);
  // Clamp.set(false);
  // chassis.turn_to_angle(-75);
  // chassis.drive_distance(-25);
  // Clamp.set(true);
  Hooks.spin(reverse);
}

void Ring_Rush_Blue(){
  odom_constants();
  chassis.set_coordinates(0,0,0);
  // default_constants();
  Intake.setVelocity(100, percent);
  Hooks.setVelocity(100, percent);
  chassis.drive_max_voltage = 12;
  RushMech.set(true);
  chassis.drive_distance(37);
  chassis.drive_max_voltage = 8;

  chassis.turn_to_angle(-30);
  chassis.drive_distance(-12);
  RushMech.set(false);
  chassis.drive_distance(-8);
  chassis.set_turn_constants(6, .4, .03, 3, 15);
  chassis.turn_to_angle(95);
  chassis.set_drive_exit_conditions(1.5, 300, 600);
  chassis.drive_distance(-28);
  Clamp.set(true);
  chassis.turn_to_angle(58);
  chassis.drive_distance(8);
  chassis.set_drive_exit_conditions(1.5, 300, 4000);
  chassis.drive_max_voltage = 4;
  Intake.spin(reverse);
  Hooks.spin(reverse);
  chassis.drive_distance(33);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(-10);
  chassis.turn_to_angle(175);
  chassis.drive_distance(43);
  chassis.turn_to_angle(-10);
  chassis.drive_distance(30);

}
void Ring_Rush_Red(){
  task intakeTask(BIntakeTask);
  //task hooksRev(hooks_rev);
  odom_constants();
  chassis.set_coordinates(0,0,0);
  // default_constants();
  Intake.setVelocity(100, percent);
  Hooks.setVelocity(100, percent);
  chassis.drive_max_voltage = 12;
  RushMech.set(true);
  chassis.drive_distance(37);
  chassis.drive_max_voltage = 8;

  chassis.turn_to_angle(30);
  chassis.drive_distance(-12);
  RushMech.set(false);
  chassis.drive_distance(-8);
  chassis.set_turn_constants(6, .4, .03, 3, 15);
  chassis.turn_to_angle(-95);
  chassis.set_drive_exit_conditions(1.5, 300, 600);
  chassis.drive_distance(-28);
  Clamp.set(true);
  chassis.turn_to_angle(-58);
  chassis.drive_distance(8);
  chassis.set_drive_exit_conditions(1.5, 300, 4000);
  chassis.drive_max_voltage = 4;
  Intake.spin(reverse);
  Hooks.spin(reverse);
  chassis.drive_distance(33);
  chassis.drive_max_voltage = 8;
  chassis.drive_distance(-10);
  chassis.turn_to_angle(-175);
  chassis.drive_distance(43);
  chassis.turn_to_angle(10);
  chassis.drive_distance(30);
  // Clamp.set(true);
  // chassis.turn_max_voltage = 6;
  // chassis.turn_to_angle(-60);
}

void Skills(){

  std::cout << "Running skills" << std::endl;
  std::cout << "Intake temp: " << Intake.temperature() << " Hooks temp: " << Hooks.temperature() << " Arm temp: " << arm.temperature() << std::endl;
  std::cout << "Chassis temp:\n" << RF.temperature() << " " << LF.temperature() << "\n" << RM.temperature() << " " << LM.temperature() << "\n" << RB.temperature() << " " << LB.temperature() << std::endl;
  // setup
  // thread intakeRev(intake_rev_if_stuck); // set up a seperate thread to reverse the intake if it gets stuck
  chassis.set_coordinates(0, 0, 0);
  odom_constants();
  Intake.setVelocity(100, percent);
  Hooks.setVelocity(100, percent);
  arm.setVelocity(100,percent);
  chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  chassis.set_turn_constants(8, .4, .03, 3, 15);
  // put ring on stake
  arm.spinFor(forward, .5, sec);
  arm.spinFor(reverse, .5, sec);
  // drive to get mobile goal
  chassis.drive_distance(-13);
  chassis.turn_to_angle(-90);
  chassis.set_drive_exit_conditions(1.5, 300, 750);
  chassis.drive_distance(-22);
  chassis.set_drive_exit_conditions(1.5, 300, 3000);
  Clamp.set(true);
  // turn to get first ring
  // chassis.turn_to_angle(0);
  Hooks.spinFor(1,turns);
  Intake.spin(reverse);
  Hooks.spin(reverse);
  chassis.set_turn_exit_conditions(1, 300, 1000);
  chassis.turn_to_angle(180);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.drive_to_point(25,-36);
  // turn to second ring
  chassis.turn_to_angle(90);
  chassis.drive_to_point(54, -36);
  // turn to third ring
  log();
  chassis.turn_to_angle(0);
  chassis.drive_to_point(47, -14);
  log();
  wait(1, seconds);
  Hooks.spinFor(fwd,1,turns);
  Hooks.spin(reverse);
  log();
  // drive to fourth ring
  chassis.drive_to_point(47.7, 1);
  // turn to fifth ring
  chassis.set_swing_exit_conditions(1, 300, 1000);
  chassis.right_swing_to_angle(135);
  chassis.drive_to_point(59, -16);
  // drop mobile goal in corner
  chassis.turn_to_angle(200);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(-15);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  Clamp.set(false);
  Intake.stop();
  Hooks.stop();
  chassis.drive_distance(10);
  log();
  // drive to get second mobile goal
  chassis.turn_to_angle(90);
  odom_constants();
  chassis.set_drive_exit_conditions(1.5, 300, 3000);
  // chassis.drive_to_point(26,15);
  log();
  chassis.drive_to_point(-28,-11);
  log();
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  Clamp.set(true);
  // Intake.spin(reverse);
  // turn to sixth ring
  chassis.turn_to_angle(180);
  Hooks.spinFor(1,turns);
  Intake.spin(reverse);
  Hooks.spin(reverse);
  chassis.drive_to_point(-25,-35);
  // turn to 7 ring
  chassis.turn_to_angle(-90);
  chassis.drive_to_point(-50.615,-38.838);
  // turn to 8 ring
  chassis.turn_to_angle(0);
  chassis.drive_to_point(-48.03,-15.873);
  // 9th ring
  wait(1, seconds);
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_to_point(-45,-1.5);
  chassis.set_drive_exit_conditions(1.5, 300, 3000);
   //10th ring
  chassis.set_swing_exit_conditions(1, 300, 1000);
  chassis.left_swing_to_angle(105);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.drive_to_pose(61.976,16.452,90);
  return;
  //put the goal in corner
  chassis.turn_to_angle(-20);
  Intake.stop();
  chassis.set_drive_exit_conditions(1.5, 300, 500);
  chassis.drive_distance(-15);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  Clamp.set(false);
  chassis.drive_distance(9);
  log();
  //drive to middle
  chassis.drive_to_point(46.39, 62.302);
  //pick up 11th ring and put on alliance stake
  Intake.spinFor(reverse,8,turns,false);
  chassis.drive_to_point(20, 90.831);
  chassis.turn_to_angle(180);
  chassis.drive_distance(-25);
  // Clamp.set(true);
  chassis.turn_to_angle(-135);
  chassis.set_drive_exit_conditions(1.5, 300, 1500);
  chassis.drive_to_point(58.9,127.089);
  chassis.drive_distance(10);
  log();
  //grab 4th goal
  // chassis.drive_to_point(-60.504,123.257);
  // chassis.drive_distance(-10);
  chassis.turn_to_angle(75);
  chassis.drive_to_point(-6, 112.638);
  //chassis.drive_distance(-30);
  Clamp.set(true);
  //ring 12
  chassis.turn_to_angle(220);
  Intake.spin(reverse);
  chassis.drive_to_point(-29.758, 92);
  // //ring 13
  chassis.drive_to_point(-49.986, 92);
  //put goal in corner
  chassis.turn_to_angle(200);
  Clamp.set(false);
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_distance(-20);
  chassis.drive_distance(20);
  log();


  

}
void odom_test(){
chassis.set_coordinates(0, 0, 0);
odom_constants();
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(0,50, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(0,70, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(0,90, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(0,110, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(0,130, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}
void odom_test2(){
chassis.set_coordinates(0, 0, 0);
odom_constants();
chassis.drive_to_point(24,24);

}

void odom_test3(){

  // setup
  chassis.set_coordinates(0, 0, 0);
  odom_constants();
  Brain.Screen.clearScreen();
  std::cout << "Odom test 3" << std::endl;

  // 1st test
  std::cout << "Starting stationary drift test in ";
  for (int i = 3; i > 0; i--) {
    Brain.Screen.setFont(fontType::prop60);
    Brain.Screen.printAt(0,60,"DO NOT MOVE THE ROBOT");
    Brain.Screen.setFont(fontType::prop20);
    Brain.Screen.printAt(0,120,"Starting in ", i);
    std::cout << i << "... ";
    Controller1.rumble(".");
    task::sleep(1000);
  }
  std::cout << std::endl;

  float chassisStartX = chassis.get_X_position();
  float chassisStartY = chassis.get_Y_position();
  float chassisStartAngle = chassis.get_absolute_heading();
  task::sleep(1000);
  float chassisEndX = chassis.get_X_position();
  float chassisEndY = chassis.get_Y_position();
  float chassisEndAngle = chassis.get_absolute_heading();


}

void skills2(){
chassis.set_coordinates(0, 0, 0);
odom_constants();

}
