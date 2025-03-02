#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RF                   motor         18              
// RB                   motor         19              
// LF                   motor         10              
// LB                   motor         6               
// Inertial             inertial      20              
// rotation1            rotation      1               
// Controller1          controller                    
// LM                   motor         8               
// RM                   motor         3               
// Hang                 digital_out   A               
// ArmPiviot            digital_out   C               
// Doinker              digital_out   B               
// Clamp                digital_out   E               
// arm                  motor         21              
// Intake               motor         17  
//            
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;
competition Competition;

#include <thread>
#include <future>
#include <iostream>
#include <cmath>
#include <list>
#include <vector>

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors using the V5 port icon in the top right of the screen. Doing     */
/*  so will update robot-config.cpp and robot-config.h automatically, so     */
/*  you don't have to. Ensure that your motors are reversed properly. For    */
/*  the drive, spinning all motors forward should drive the robot forward.   */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your robot manually with the sidebar configurer. */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Specify your drive setup below. There are seven options:
//ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
//For example, if you are not using odometry, put ZERO_TRACKER_NO_ODOM below:
TANK_TWO_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LF, LM, LB),

//Right Motors:
motor_group(RF,RM, RB),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT1,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
2.75,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
1,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
356.13,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
1,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT5,

//Sideways tracker diameter (reverse to make the direction switch):
-2,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
.5

);

int Bintaketask(){
  uint32_t lastCheckTime = vex::timer::system();
  ColorSort.integrationTime(50);
  ColorSort.setLightPower(100);

  while(true){
    //Hooks.setVelocity(-100,percent);
    // Intake.setVelocity(-100,percent);

    int hue = ColorSort.hue();
    if(((hue >= 200 && hue <= 260))){
      Brain.Screen.print("Blue detected! Spinning hooks.");
      Brain.Screen.newLine();
      std::cout << "Blue detected! Spinning hooks.\n";
      Hooks.setVelocity(100,percent);
      Intake.setVelocity(100,percent);
      this_thread::sleep_for(150);
      Hooks.setVelocity(-100,percent);
      Intake.setVelocity(-100,percent);
    }
    this_thread::sleep_for(50);
  }
}

int Rintaketask(){
  uint32_t lastCheckTime = vex::timer::system();
  ColorSort.setLightPower(100);
  ColorSort.integrationTime(50);
  while(true){
    // Hooks.setVelocity(-100,percent);
    // Intake.setVelocity(-100,percent);

    int hue = ColorSort.hue();
    if(((hue >= 0 && hue <= 38))){
      Brain.Screen.print("Red detected! Spinning hooks.");
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

void virtual_field_on_screen(double startX = 0, double startY = 0) { // draws a virtual field if you want that for some reason. use startX and startY to offset the starting position
  Brain.Screen.clearScreen(); // clear brain screen so nothing gets in the way of the virtual field
  int oldX = chassis.get_X_position(); // oldX and oldY don't start with values at the beginning of the while loop, so they're set here
  int oldY = chassis.get_Y_position();
  int curX;
  int curY;
  int visualSize = 10; // width/length in pixels of the square that all objects are displayed as
  enum vShape {
    square, circle, donut, stake, goal
  };
  class vObj { // defining the virtual object to make the list easier to use
    public:
      int X = 0; // X position
      int Y = 0; // Y position
      int Color = color::red; // Color
      int Shape = vShape::square; // Shape if drawing a different shape is desired
      vObj(int x, int y, int color = color::red, int shape = vShape::square) { // constructor
        X = x;
        Y = y;
        Color = color;
        Shape = shape;
      };
      void draw(int visualSize) { // this function draws the shape
        Brain.Screen.setFillColor(Color); // set fill color to the desired color
        switch (Shape) {
        case vShape::square:
          Brain.Screen.drawRectangle(X - visualSize/2, Y - visualSize/2,visualSize,visualSize); // draws a square that is centered around the coordinates of the object
          break;
        case vShape::circle: 
          Brain.Screen.drawCircle(X, Y,visualSize);
          break;
        case vShape::donut:
          Brain.Screen.drawCircle(X, Y,visualSize);
          Brain.Screen.setFillColor(color::black);
          Brain.Screen.drawCircle(X, Y,visualSize/2);
          break;
        case vShape::stake:
          Brain.Screen.drawRectangle(X - visualSize/2, Y - visualSize/6,visualSize,visualSize/3);
          Brain.Screen.drawRectangle(X - visualSize/6, Y - visualSize,visualSize/3,visualSize);
          break;
        case vShape::goal:
          Brain.Screen.drawRectangle(X - visualSize/2, Y - visualSize/4,visualSize,visualSize/2);
          Brain.Screen.drawRectangle(X - visualSize/4, Y - visualSize,visualSize/2,visualSize);
          break;
        }
      }
  };
  
  // below is the list that holds every object that is to be displayed
  std::list<vObj*> objList = {/*add objects to this list via the following: new vObj(x position, y position, color (optional), shape (optional))*/new vObj(0,0)};
  for (auto i : objList) { // display all objects ahead of time
    i->draw(visualSize);
  }
  while (1) {
    // set current coordinates for later use
    curX = chassis.get_X_position();
    curY = chassis.get_Y_position();
    // the below function is crucial for optimization, as drawing pixels takes up a significant amount of time and may cause flickering
    for (auto i : objList) { // runs through the objects list to see if any need to be updated
      if (abs(oldX-i->X) <= visualSize || abs(oldY-i->Y) <= visualSize) { // detects if the object was being intercected by the robot and if so, redraws itself
        i->draw(visualSize);
      }
    }
    // erase where the robot was
    Brain.Screen.setFillColor(color::black);
    Brain.Screen.drawRectangle(oldX + startX - visualSize/2, oldY + startY - visualSize/2,visualSize,visualSize);
    // draw where the robot now is
    Brain.Screen.setFillColor(color::white);
    Brain.Screen.drawRectangle(curX + startX - visualSize/2, curY + startY - visualSize/2,visualSize,visualSize);
    // save old coordinates for drawing purposes
    oldX = curX;
    oldY = curY;
    task::sleep(200);
  }
}

void tetris() { // tetris!!!
  class shape { // i made a shape class because i thought it would be fun, there's not really a reason for this
    public:
    shape(std::list<int> pos1, std::list<int> pos2, std::list<int> pos3, std::list<int> pos4) { // shape constructor
      X1 = pos1.front();
      Y1 = pos1.back();
      X2 = pos2.front();
      Y2 = pos2.back();
      X3 = pos3.front();
      Y3 = pos3.back();
      X4 = pos4.front();
      Y4 = pos4.back();
    }
    std::list<int> Pos1_rotated(int rot) { // rotates the positions for when a shape is rotated, simple enough
      return {X1 * (int)cos(rot*90) + Y1 * (int)sin(rot*90), X1 * (int)sin(rot*90) + Y1 * (int)cos(rot*90)};
    }
    std::list<int> Pos2_rotated(int rot) {
      return {X2 * (int)cos(rot*90) + Y2 * (int)sin(rot*90), X2 * (int)sin(rot*90) + Y2 * (int)cos(rot*90)};
    }
    std::list<int> Pos3_rotated(int rot) {
      return {X3 * (int)cos(rot*90) + Y3 * (int)sin(rot*90), X3 * (int)sin(rot*90) + Y3 * (int)cos(rot*90)};
    }
    std::list<int> Pos4_rotated(int rot) {
      return {X4 * (int)cos(rot*90) + Y4 * (int)sin(rot*90), X4 * (int)sin(rot*90) + Y4 * (int)cos(rot*90)};
    }
    private: // there are so many variables here
    int X1 = 0;
    int Y1 = 0;
    int X2 = 0;
    int Y2 = 0;
    int X3 = 0;
    int Y3 = 0;
    int X4 = 0;
    int Y4 = 0;
  };
  std::vector<std::vector<int>> block_list {}; // list of all placed blocks
  block_list.resize(8); // this sets the height
  for (auto i : block_list) { // this sets the width
    i.resize(8);
  }
  std::vector<std::vector<int>> old_list = block_list; // save the old list
  std::vector<uint32_t> block_color {color::red,color::orange,color::yellow,color::green,color::blue,color::purple}; // list of colors
  std::vector<shape*> shape_list = {new shape({0,1},{0,0},{0,-1},{0,-2}),new shape({0,1},{0,0},{0,-1},{0,-2}),new shape({0,1},{0,0},{0,-1},{0,-2}),new shape({0,1},{0,0},{0,-1},{0,-2}),new shape({0,1},{0,0},{0,-1},{0,-2}),new shape({0,1},{0,0},{0,-1},{0,-2}),new shape({0,1},{0,0},{0,-1},{0,-2}),}; // list of shapes
  enum move_goal { // different ways the held piece can move
    none, moveDown, moveLeft, moveRight
  };
  enum shape_types { // different types of shape
    line, square, t_piece, l_piece, rev_l_piece, s_piece, z_piece
  };
  move_goal move_goal = move_goal::none; // current way the held piece is trying to move
  int rot = 0; // piece rotation
  int piece = 0; // curently held piece
  int next_piece = 0; // next piece
  int score = 0; // score
  int score_adding = 0; // used for the score bonus you get from clearing multiple rows
  int i = 0;
  int j = 0;
  int x = 4;
  int y = 0;
  int move_frames = 30; // how many frames until the piece falls
  bool clear_row = false;
  bool pressing_key = false;
  bool can_move_right = true;
  bool can_move_left = true;
  auto place_shape = [=, &block_list, &piece, &next_piece](int x, int y){ // lambada that places shapes at a specified location
    shape* Ishape = shape_list.at(piece); // gets the shape that corresponds the the held piece
    std::list<int> pos1 = Ishape->Pos1_rotated(rot); // get rotated positions
    std::list<int> pos2 = Ishape->Pos2_rotated(rot);
    std::list<int> pos3 = Ishape->Pos3_rotated(rot);
    std::list<int> pos4 = Ishape->Pos4_rotated(rot);
    block_list.at(pos1.back()+y).at(pos1.front()+x)=piece; // set the spots where the blocks should be to the piece
    block_list.at(pos2.back()+y).at(pos2.front()+x)=piece;
    block_list.at(pos3.back()+y).at(pos3.front()+x)=piece;
    block_list.at(pos4.back()+y).at(pos4.front()+x)=piece;
    piece = next_piece; // set the piece to the next piece
    next_piece = rint(6); // set the next piece to a random piece
    if (next_piece >= 5 && piece >= 5) { // fun fact: the original tetris doesn't let you get two s/z pieces in a row
      next_piece = rint(4);
    }
  };
  Brain.Screen.clearScreen(); // clear screen so that we aren't drawing over anything
  while (1) { // while loop so that the game actually runs
    std::cout << move_goal;
    if (move_goal != move_goal::none || move_frames == 30) {
      shape* Shape = shape_list.at(piece); // get the shape
      std::list<int> pos1 = Shape->Pos1_rotated(rot);
      std::list<int> pos2 = Shape->Pos2_rotated(rot);
      std::list<int> pos3 = Shape->Pos3_rotated(rot);
      std::list<int> pos4 = Shape->Pos4_rotated(rot);
      Brain.Screen.setFillColor(block_color.at(piece));
      Brain.Screen.drawRectangle(pos1.back()*24,pos1.front()*24,24,24);
      Brain.Screen.drawRectangle(pos2.back()*24,pos2.front()*24,24,24);
      Brain.Screen.drawRectangle(pos3.back()*24,pos3.front()*24,24,24);
      Brain.Screen.drawRectangle(pos4.back()*24,pos4.front()*24,24,24);
      if (block_list.at(pos1.back()+1).at(pos1.front()) != 0 || block_list.at(pos2.back()+1).at(pos2.front()) != 0 || block_list.at(pos3.back()+1).at(pos3.front()) != 0 || block_list.at(pos4.back()+1).at(pos4.front()) != 0) {
        place_shape(x,y);
      }
      if (block_list.at(pos1.back()).at(pos1.front()+1) != 0 || block_list.at(pos2.back()).at(pos2.front()+1) != 0 || block_list.at(pos3.back()).at(pos3.front()+1) != 0 || block_list.at(pos4.back()).at(pos4.front()+1) != 0) {
        can_move_right = false;
      }
      else {
        can_move_right = true;
      }
      if (block_list.at(pos1.back()).at(pos1.front()-1) != 0 || block_list.at(pos2.back()).at(pos2.front()-1) != 0 || block_list.at(pos3.back()).at(pos3.front()-1) != 0 || block_list.at(pos4.back()).at(pos4.front()-1) != 0) {
        can_move_left = false;
      }
      else {
        can_move_left = true;
      }
    }
    if (block_list != old_list) {
      old_list = block_list;
      i = 0;
      for (auto row : block_list) {
        i++;
        if (row != old_list.at(i)) {
          j = 0;
          clear_row = true;
          for (auto block : row) {
            j++;
            if (block == 0) {
              clear_row = false;
            }
            if (block != old_list.at(i).at(j)) { 
              Brain.Screen.setFillColor(block_color.at(block));
              Brain.Screen.drawRectangle(j*24,i*24,24,24);
            }
          }
          if (clear_row) {
            row.~vector();
            block_list.at(i).~vector();
            block_list.resize(8);
            score_adding *= 2;
            if (score_adding == 0) {
              score_adding = 2;
            }
          }
        }
      }
      if (score_adding != 0) {
        score += score_adding/2;
        score_adding = 0;
      }
    }
    switch (move_goal) {
      case move_goal::none:
        move_frames-=1;
        break;
      case move_goal::moveDown:
        y+=1;
        move_frames = 30;
        break;
      case move_goal::moveLeft:
        if (can_move_left) {
          x-=1;
        }
        break;
      case move_goal::moveRight:
        if (can_move_right) {
          x+=1;
        }
        break;
    }
    move_goal = move_goal::none;
    if (!pressing_key) {
      if (Controller1.ButtonDown.pressing() || (Brain.Screen.pressing() && Brain.Screen.yPosition() < 240) || move_frames <= 0) {
        pressing_key = true;
        move_goal = move_goal::moveDown;
      }
      else if (Controller1.ButtonLeft.pressing() || (Brain.Screen.pressing() && Brain.Screen.xPosition() < 240)) {
        pressing_key = true;
        move_goal = move_goal::moveLeft;
      }
      else if (Controller1.ButtonRight.pressing() || (Brain.Screen.pressing() && Brain.Screen.xPosition() >= 240)) { 
        pressing_key = true;
        move_goal = move_goal::moveRight;
      }
    }
    if (Controller1.ButtonDown.pressing() || Controller1.ButtonLeft.pressing() || Controller1.ButtonRight.pressing() || Brain.Screen.pressing()) {
      pressing_key = true;
    }
    else {
      pressing_key = false;
    }
    task::sleep(40);
  }
}

enum class ArmState {
Down,
Ring1,
Hang,
Wall
};

ArmState currentState = ArmState::Down;

// Define the angles for each state
const int Down_Angle = -1;
const int Ring1_Angle = -145;
const int Hang_Angle = -70;
const int Wall_Angle = -675;

bool colorSortRed = true;
int colorSortMin = 0;
int colorSortMax = 40;

int bintaketask(){
  ColorSort.integrationTime(20);
  ColorSort.setLightPower(100);
  while(true){
    int hue = ColorSort.hue();
    if (hue >= 200 && hue <= 260 && ColorSort.isNearObject()) {
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

int rintaketask(){
 uint32_t lastCheckTime = vex::timer::system();
  ColorSort.integrationTime(50);
  ColorSort.setLightPower(100);
  while(true){
    int hue = ColorSort.hue();
    if(((hue >= 0 && hue <= 40) || hue >= 340) && ColorSort.isNearObject()){
      Brain.Screen.print("Red detected! Spinning hooks.");
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


void switchColorSortValues() {
  colorSortRed = !colorSortRed;
  if (colorSortRed) {
    colorSortMin = 0;
    colorSortMax = 40;
  }
  else {
    colorSortMin = 200;
    colorSortMax = 260;
  }
}

// Function to set the arm angle
void setArmAngle(int angle) {
  arm.setVelocity(60, percent);
  arm.spinToPosition(angle, degrees); // Adjust the speed as necessary
}

// Function to get the current arm angle
int getArmAngle() {
  return rotation1.position(degrees); // Assuming the motor has an encoder
}
  
void updateArmState() {
  if (Controller1.ButtonA.pressing()) {
    currentState = ArmState::Down;
  } else if (Controller1.ButtonB.pressing()) {
    currentState = ArmState::Ring1;
  } /*
  else if (Controller1.ButtonX.pressing()) {
     currentState = ArmState::Hang;
  }*/
  else if (Controller1.ButtonY.pressing()) {
    currentState = ArmState::Wall;
  }
}

// Function to correct the arm angle if it deviates from the target
void correctArmAngle() {
  double targetAngle;
  switch (currentState) {
    case ArmState::Down:
      targetAngle = Down_Angle;
      break;
    case ArmState::Ring1:
      targetAngle = Ring1_Angle;
      break;
     case ArmState::Hang:
       targetAngle = Hang_Angle;
     break;
       case ArmState::Wall:
      targetAngle = Wall_Angle;
      break;
  }
  targetAngle = -targetAngle;
  double currentAngle = /*rotation1.position(deg) || */arm.position(deg);
  /*
  //if (!arm.isSpinning()) {
    arm.spinToPosition(targetAngle,degrees,false);
  //}
  
  if (currentAngle != targetAngle) {
    //setArmAngle(targetAngle);
  }
  */
  double speed = .5;
  double margin_of_error = .5;
  arm.setVelocity((std::abs(targetAngle - currentAngle) > margin_of_error) * speed * (targetAngle - currentAngle), percent);
  
}

// void ColorSortBlue(){
//   color detectedColor = ColorSort.color();
// if (detectedColor == red){
//   Hooks.stop(brake);
// }
// if (detectedColor == blue)
//   Hooks.setVelocity(100,percent);
// }

// void ColorSortRed(){
//   color detectedColor = ColorSort.color();
// if (detectedColor == blue){
//   Hooks.stop(brake);
// }
// if (detectedColor == red)
//   Hooks.setVelocity(100,percent);
// }

int current_auton_selection = 0;
bool auto_started = false;

void pre_auton(void) {
  vexcodeInit();
  default_constants();

  while(auto_started == false){            
    Brain.Screen.clearScreen();            
    switch(current_auton_selection){      
      case 0:
        Brain.Screen.printAt(50, 50, "Blue Solo WP");
        break;
      case 1:
        Brain.Screen.printAt(50, 50, "Red Solo WP");
        break;
      case 2:
        Brain.Screen.printAt(50, 50, "Blue 4 Ring");
        break;
      case 3:
        Brain.Screen.printAt(50, 50, "Red 4 Ring ");
        break;
      case 4:
        Brain.Screen.printAt(50, 50, "Blue Goal Rush");
        break;
      case 5:
        Brain.Screen.printAt(50, 50, "Red Goal Rush");
        break;
      case 6:
        Brain.Screen.printAt(50, 50, " Ring Rush Blue");
        break;
      case 7:
        Brain.Screen.printAt(50, 50, "Ring Rush Red");
        break;
      case 8:
        Brain.Screen.printAt(50, 50, "Skills");
        break;
      case 9:
      Brain.Screen.printAt(50, 50, "Null");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 9){
      current_auton_selection = 0;
    }
    if (!Controller1.installed() && !Competition.isFieldControl()) {
      Brain.Screen.setFillColor(color::green);
      Brain.Screen.drawRectangle(0,200,480,180);
      Brain.Screen.printAt(80,220,"Go To Mode Select");
      Brain.Screen.setFillColor(color::transparent);
      if (Brain.Screen.yPosition() > 200) {
        return;
      }
    }
    task::sleep(10);
  }
}

void autonomous(void) {
  Brain.Timer.reset();
  auto_started = true;
  switch(current_auton_selection){  
    case 0: 
      Blue_Solo_WP(); //This is the default auton, if you don't select from the brain.
      break;        //Change these to be your own auton functions in order to use the auton selector.
    case 1:         //Tap the screen to cycle through autons.
      Red_Solo_WP();
      break;
    case 2:
      Blue_4_Ring();
      break;
    case 3:
      Red_4_Ring();
      break;
    case 4:
      Blue_Goal_Rush();
      break;
    case 5:
      Red_Goal_Rush();
      break;
    case 6:
      Ring_Rush_Blue();
      break;
    case 7:
      Ring_Rush_Red();
      break;
    case 8:
      Skills();
      break;
 }
//  while(true){
//   if (current_auton_selection == 0 or current_auton_selection == 2 or current_auton_selection == 5){
//       ColorSortBlue();
//     }
//   if (current_auton_selection == 1 or current_auton_selection == 3 or current_auton_selection == 4 or current_auton_selection == 6){
//       ColorSortRed();
//     }
//  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  bool controller_vibrated = false;
  bool buttonL2pressing = false;
  bool pnuematicsPressing = false;
  bool mogo = false;
  bool doinker = false;
  bool rushmech = false;
  bool isCompetition = (Competition.isFieldControl() || Competition.isCompetitionSwitch());
  Brain.resetTimer();
  arm.spin(forward);
  arm.setVelocity(0, percent);
  arm.setStopping(hold);
  // auto_started = true;
  // odom_test();
  // return;

  if (Controller1.ButtonL2.pressing()) {
    tetris();
  }

  if (current_auton_selection/2 == round(current_auton_selection/2)) {
    switchColorSortValues();
  }

  //task ringIntakeTask(Bintaketask);

  while (1) {
    

    chassis.control_tank();

  if (!controller_vibrated && Brain.Timer.value() >= 70 && isCompetition) {
    controller_vibrated = true;
    Controller1.rumble(".-");
  } 

    //vex::task intakeTask(rintaketask);
    // std::cout << Brain.Timer.time();
    // if (Controller1.ButtonRight.pressing()){
    //   Doinker.set(false);
    //   } 
    //  if (Controller1.ButtonLeft.pressing()){
    //   Doinker.set(true);
    //   } 
    updateArmState();
    correctArmAngle();
      
      if (Controller1.ButtonL2.pressing()){
        buttonL2pressing = true;
      }
      else if (buttonL2pressing){
        std::cout << buttonL2pressing << "\n";
        buttonL2pressing = false;
        Hooks.setVelocity(100,pct);
        Hooks.spin(forward);
        Hooks.spinFor(90,degrees,false);
      }
      else if (Controller1.ButtonL1.pressing()){
        Intake.setVelocity(100, percent);
        Intake.spin(reverse);
        Hooks.stop(coast);
      }
      else if (Controller1.ButtonR2.pressing()){
        Intake.setVelocity(100, percent);
        Intake.spin(forward);
        Hooks.setVelocity(100, percent);
        Hooks.spin(forward);
      }
      else if (Controller1.ButtonR1.pressing()){
        Intake.setVelocity(100, percent);
        Intake.spin(reverse);
        Hooks.setVelocity(100, percent);
        Hooks.spin(reverse);
      }
      else{
        Intake.stop(coast);
        Hooks.stop(coast);
      }

    //   if  (Controller1.ButtonUp.pressing()){
    //     Clamp.set(false);
    //   }

    //   if (Controller1.ButtonDown.pressing()){
    //     Clamp.set(true);
    //  }

    if (Controller1.ButtonUp.pressing()){
    while(Controller1.ButtonUp.pressing()){
    this_thread::sleep_for(1);
    }
    mogo = !mogo;
    Clamp.set(mogo);
    }

    if (Controller1.ButtonRight.pressing()){
    while(Controller1.ButtonRight.pressing()){
    this_thread::sleep_for(1);
    }
    doinker = !doinker;
    Doinker.set(doinker);
    }

    if (Controller1.ButtonLeft.pressing()){
    while(Controller1.ButtonLeft.pressing()){
    this_thread::sleep_for(1);
    }
    rushmech = !rushmech;
    RushMech.set(rushmech);
    }

    this_thread::sleep_for(20);
  }
}

//
// Main will set up the competition functions and callbacks.
//

int main() {
  /*Controller1.ButtonL2.pressed([](){
    Hooks.spinFor(90,degrees,false);
  });*/
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  if (!Controller1.installed() && !Competition.isFieldControl()) {
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(color::blue);
    Brain.Screen.drawRectangle(0,0,240,240);
    Brain.Screen.printAt(40,120,"Run Driver Control");
    Brain.Screen.setFillColor(color::red);
    Brain.Screen.drawRectangle(240,0,240,240);
    Brain.Screen.printAt(280,120,"Run Autonomous");
    while (Brain.Screen.pressing()) {
      task::sleep(20);
    }
    while (!Brain.Screen.pressing()) {
      task::sleep(20);
    }
    Brain.Screen.clearScreen();
    if (Brain.Screen.xPosition() > 240) {
      Competition.test_auton();
    }
    else {
      Competition.test_driver();
    }
  }

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
