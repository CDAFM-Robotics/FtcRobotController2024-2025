package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Driver Control TeleOp", group = "0competition")
@Config
public class DriverControlledOpMode extends LinearOpMode {
  // Define all the states for the arm and hand
  public enum ArmHandState {
    ROT_ARM_HOLD,
    ROT_DRIVER_CONTROL,
    ARM_PICKUP,
    HAND_PICKUP_SEQ,
    HAND_PICKUP_SEQ_1,
    HAND_PICKUP_SEQ_2,
    HAND_PICKUP_SEQ_3,
    ARM_DROP_TOP,
    ARM_DROP_BOTTOM,
    HAND_DROP,
    HAND_DROP_1,
    ARM_HAND_DRIVE,
    ARM_TOP_SPECIMEN,
    ARM_TOP_SPECIMEN_PULL,
    ARM_DROP_READY
  }

  // Current arm and hand state and previous arm state
  ArmHandState armHandState;
  ArmHandState prevArmHandState;

  

  // Define a timer
  ElapsedTime timer = new ElapsedTime();
  int tickPerCycle;

  ElapsedTime hangTimer = new ElapsedTime();

  //Create an instant of the Robot Class
  Robot robot = new Robot(this);

  // define the current and previous gamepads
  public Gamepad currentGamepad1 = new Gamepad();
  public Gamepad previousGamepad1 = new Gamepad();
  public Gamepad currentGamepad2 = new Gamepad();
  public Gamepad previousGamepad2 = new Gamepad();

  // over ride the runOpMode function from LinearOpMode
  @Override
  public void runOpMode() {

    telemetry.addData("Status", "Initializing...");
    // Initialize the robot
    robot.initializeDevices();

    // Initialize the arm and hand state
    armHandState = ArmHandState.ROT_ARM_HOLD;

    telemetry.addData("Status", "Initialized");
    telemetry.addData("Rotation Motor ", "Target: %d, Current: %d", robot.getSlideRotationMotorTargetPosition(), robot.getSlideRotationMotorCurrentPosition());
    telemetry.addData("Extension Motor ", "Target: %s, Current: %d", robot.getSlideExtensionMotorTargetPosition(), robot.getSlideExtensionMotorCurrentPosition());
    telemetry.update();

    while (opModeInInit()) {
      robot.team = Robot.Team.RED;
    }

    waitForStart();

    hangTimer.reset();

    while (opModeIsActive()) {

      telemetry.addData("Status", "Running");
      // Update the gamepads.
      // Using previousGamepad and currentGampad to detect one button push.
      previousGamepad1.copy(currentGamepad1);
      currentGamepad1.copy(gamepad1);
      previousGamepad2.copy(currentGamepad2);
      currentGamepad2.copy(gamepad2);

      // Drive Train Control
      if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
        robot.toggleDriveSpeed();
      }
      telemetry.addData("Speed", robot.getDriveSpeed());
      robot.setMotorPowers(Math.pow(currentGamepad1.left_stick_x, 3), Math.pow(currentGamepad1.left_stick_y, 3), Math.pow(currentGamepad1.right_stick_x, 3), 0);

      /*************************
       *  Driver Arm Control   *
       *************************/
      // Driver Arm Rotation Control
      if (currentGamepad2.left_stick_y < 0) {
        robot.setSlideRotationMotorPower(Math.pow(currentGamepad2.left_stick_y,2));
        robot.setSlideRotationMotorTargetPosition(Robot.ARM_ROT_DROP_OFF_SAMPLES);
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ROT_DRIVER_CONTROL;
      }
      else if (currentGamepad2.left_stick_y > 0) {
        robot.setSlideRotationMotorPower(Math.pow(currentGamepad2.left_stick_y,2));
        robot.setSlideRotationMotorTargetPosition(Robot.ARM_ROT_INIT);
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ROT_DRIVER_CONTROL;
      }
      else {
        if (currentGamepad2.left_stick_y == 0 && previousGamepad2.left_stick_y != 0 ) {
          robot.setSlideRotationMotorPower(Robot.ARM_ROT_POWER_FULL);
          robot.setSlideRotationMotorTargetPosition(robot.getSlideRotationMotorCurrentPosition());
          prevArmHandState = armHandState;
          armHandState = ArmHandState.ROT_ARM_HOLD;
        }
      }

      // Driver Arm Extension Control
      if (currentGamepad2.right_stick_y < 0) {
        robot.setSlideExtensionMotorPower(currentGamepad2.right_stick_y * Math.pow(currentGamepad2.right_stick_y,2));
        robot.setSlideExtMotorTargetPosWithLimit(Robot.ARM_EXT_DROP_TOP_BASKET);
      }
      else if (currentGamepad2.right_stick_y > 0) {
        robot.setSlideExtensionMotorPower(currentGamepad2.right_stick_y * Math.pow(currentGamepad2.right_stick_y,2));
        robot.setSlideExtensionMotorTargetPosition(Robot.ARM_EXT_INIT);
      }
      else {
        if (currentGamepad2.right_stick_y == 0 && previousGamepad2.right_stick_y != 0) {
          robot.setSlideExtensionMotorPower(Robot.ARM_EXT_POWER);
          robot.setSlideExtensionMotorTargetPosition(robot.getSlideExtensionMotorCurrentPosition());
        }
      }

      /*************************
       *  Driver Hand Control  *
       *************************/
      // Toggle the finger position
      if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
        robot.toggleClawGrabPosition();
      }

      // Wrist joint move up 0
      if (currentGamepad2.b && !previousGamepad2.b) {
        robot.clawPanServoUp();
      }

      // Wrist joint move down
      if (currentGamepad2.a && !previousGamepad2.a) {
        robot.clawPanServoDown();
      }
      // Wrist joint rotate left
      if (currentGamepad2.x && !previousGamepad2.x) {
        robot.clawRotateServoLeft();
      }

      // Wrist joint rotate right
      if (currentGamepad2.y && !previousGamepad2.y) {
        robot.clawRotateServoRight();
      }

      // Turn wrist joint by 90 degrees
      if (currentGamepad2.start && !previousGamepad2.start) {
        robot.toggleClawRotation();
      }

      // automatic pick up
/*    boolean sampleToPickup = robot.sampleToPickUp();

      if (sampleToPickup && robot.clawGrabServo.getPosition() == Robot.CLAW_GRAB_POSITION_OPEN && robot.isArmPickup() &&
        (armHandState != ArmHandState.HAND_PICKUP_SEQ &&
          armHandState != ArmHandState.HAND_PICKUP_SEQ_1 &&
          armHandState != ArmHandState.HAND_PICKUP_SEQ_2 &&
          armHandState != ArmHandState.HAND_PICKUP_SEQ_3)) {
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_PICKUP;
      }

      if (currentGamepad2.right_trigger > 0) {
        if (robot.isArmPickup()) {
          prevArmHandState = armHandState;
          armHandState = ArmHandState.ARM_PICKUP;
        }
        if (robot.isArmTopDropReady()) {
          prevArmHandState = armHandState;
          armHandState = ArmHandState.ARM_DROP_READY;
        }
        if (robot.isArmBottomDropReady()) {
          prevArmHandState = armHandState;
          armHandState = ArmHandState.ARM_DROP_READY;
        }
      }
      if (currentGamepad2.right_trigger > 0 || (sampleToPickup && robot.clawGrabServo.getPosition() == Robot.CLAW_GRAB_POSITION_OPEN &&
        (armHandState != ArmHandState.HAND_PICKUP_SEQ &&
          armHandState != ArmHandState.HAND_PICKUP_SEQ_1 &&
          armHandState != ArmHandState.HAND_PICKUP_SEQ_2 &&
          armHandState != ArmHandState.HAND_PICKUP_SEQ_3))) {
        switch (armHandState) {
          case ARM_PICKUP:
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ;
            // reset the timer
            timer.reset();
            break;
          case ARM_DROP_READY:
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_DROP;
            timer.reset();
            break;
          case ARM_TOP_SPECIMEN:
            robot.pullExtToHangSpecimen();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ARM_TOP_SPECIMEN_PULL;
            break;
          default:
            break;
        }
      }
*/
      /*************************
       *       Macros          *
       *************************/
      // set the arm at pick up position
      if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_PICKUP;
        robot.pickupPosition();
      }

      // drop in top basket
      if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_DROP_TOP;
        robot.dropTopBasket();
      }

      // drop in bottom basket
      if (currentGamepad2.back && !previousGamepad2.back) {
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_DROP_BOTTOM;
        robot.dropBottomBasket();
      }

      // pick up specimen for side wall
      if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
        robot.wallPickup();
      }

      // get ready to hang specimen on top bar
      if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
        robot.topSpecimenBar();
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_TOP_SPECIMEN;
      }

      // drive position
      if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
        robot.drivePosition();
        prevArmHandState = armHandState;
        armHandState = ArmHandState.ARM_HAND_DRIVE;
      }

      // geting ready to hang the robot
      if (hangTimer.seconds() >= 90) {
        if (currentGamepad1.y && !previousGamepad1.y) {
          robot.getReadyToHangRobot();
        }

        // hang the robot
        if (currentGamepad1.x && !previousGamepad1.x) {
          robot.HangRobot();
        }
      }

      //check to see if the if the pick up position if at the right height
      switch (armHandState) {
        case ARM_PICKUP:
          robot.setPickUpHeight();
        default:
      }

      // right trigger => pick up or drop off sequence
      if (currentGamepad2.right_trigger > 0 ) {
        switch (armHandState) {
          case ARM_PICKUP:
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ;
            timer.reset();
            break;
          case ARM_DROP_READY:
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_DROP;
            timer.reset();
            break;
          case ARM_TOP_SPECIMEN:
            robot.pullExtToHangSpecimen();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ARM_TOP_SPECIMEN_PULL;
            break;
          default:
            break;
        }
      }


      /*********************************************************
       *  move the arm and hand according to target positions  *
       *********************************************************/
      switch (armHandState) {
        case HAND_PICKUP_SEQ:
          if (!robot.isFingersOpen()) {
            robot.openFingers();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ_1;
            break;
          }
          else {
            robot.handPickUpdip();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ_2;
            break;
          }
        case HAND_PICKUP_SEQ_1:
          if (timer.seconds() > 0.4) {
            robot.handPickUpdip();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ_2;
            timer.reset();
            break;          }
          else {
            break;
          }
        case HAND_PICKUP_SEQ_2:
          if (timer.seconds() > 0.3) {
            robot.closeFingers();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.HAND_PICKUP_SEQ_3;
            timer.reset();
            break;          }
          else {
            break;
          }
        case HAND_PICKUP_SEQ_3:
          if (timer.seconds() > 0.4) {
            robot.handStraight();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ARM_PICKUP;
            break;
          }
          else {
            break;
          }
        case HAND_DROP:
          robot.openFingers();
          prevArmHandState = armHandState;
          armHandState = ArmHandState.HAND_DROP_1;
          break;
        case HAND_DROP_1:
          if (timer.seconds() > 0.4) {
            robot.handStraight();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ROT_ARM_HOLD;
            break;
          }
          else {
            break;
          }
        case ARM_DROP_TOP:
        case ARM_DROP_BOTTOM:
          robot.moveArmToPosition();
          robot.moveHandToPosition();
          if (robot.armReachedTarget()) {
            robot.handDropTopDip();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ARM_DROP_READY;
            break;
          }
          else
            break;
        case ARM_TOP_SPECIMEN_PULL:
          robot.moveArmToPosition();
          if(robot.armReachedTarget()) {
            robot.openFingers();
            prevArmHandState = armHandState;
            armHandState = ArmHandState.ROT_ARM_HOLD;
            break;
          }
          else
            break;
        default:
          robot.moveHandToPosition();
          robot.moveArmToPosition();
          break;
      }

      //temp test
      //robot.moveHandToPosition();
      //robot.moveArmToPosition();

      telemetry.addData("Rotation Motor ", "Target: %d, Current: %d", robot.getSlideRotationMotorTargetPosition(), robot.getSlideRotationMotorCurrentPosition());
      telemetry.addData("Rotation Motor ", "power: %f, p per cycle: %d", robot.slideRotationMotor.getPower(), tickPerCycle);
      telemetry.addData("Extension Motor ", "Target: %s, Current: %d", robot.getSlideExtensionMotorTargetPosition(), robot.getSlideExtensionMotorCurrentPosition());
      telemetry.addData("Extension Motor ", "power: %f", robot.slideExtensionMotor.getPower());
      telemetry.addData("Fingers servo ", "position %f", robot.clawGrabServo.getPosition());
      telemetry.addData("pan servo ", "position %f", robot.clawPanServo.getPosition());
      telemetry.addData("rotation servo ", "position %f", robot.clawRotateServo.getPosition());
      telemetry.addLine(armHandState.toString());

      telemetry.update();
      }
  }
}