package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RRPushTrajectories;
import org.firstinspires.ftc.teamcode.common.RRTrajectories;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Observation Zone Push Autonomous", group = "Testing")
public class AutoObservationPushSideOpMode extends LinearOpMode {

  RRPushTrajectories rrTrajectories;
  Robot robot;

  Action[] trajectories;

  @Override
  public void runOpMode() {
    robot = new Robot(this);
    rrTrajectories = new RRPushTrajectories(this.hardwareMap);

    robot.initializeArmDevices();
    //robot.slideExtensionMotor.setPower(Robot.ARM_EXT_POWER_AUTO);
    robot.slideExtensionMotor.setPower(1);
    rrTrajectories.initTrajectories();
    trajectories = rrTrajectories.getRightSideTrajectories();


    waitForStart();

    // HANG FIRST (HELD) SPECIMEN
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_HANG);

    Actions.runBlocking(rrTrajectories.rightStartToBar);

    robot.slideExtensionMotor.setPower(1.0); // TODO FULL SPEED RETRACT ON HANGPULL
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG_PULL);
    sleep(550);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    sleep(100);


    robot.slideExtensionMotor.setTargetPosition(0);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP_WALL);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_STRAIGHT);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP_WALL);

    // GO TO WALL
    Actions.runBlocking(rrTrajectories.barToObservationZoneAnd3Samples);



    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);
    sleep(400);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG + 100);
    robot.slideExtensionMotor.setPower(Robot.ARM_EXT_POWER_AUTO);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_HANG);

    Actions.runBlocking(rrTrajectories.specimenWallPosToBar);



    robot.slideExtensionMotor.setPower(1.0); // TODO FULL SPEED RETRACT
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG_PULL); // TODO
    sleep(550);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    sleep(100);

    robot.slideExtensionMotor.setTargetPosition(0);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP_WALL);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_STRAIGHT);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP_WALL);

    Actions.runBlocking(rrTrajectories.barToSpecimenWallPos);



    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);
    sleep(400);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG);
    robot.slideExtensionMotor.setPower(Robot.ARM_EXT_POWER_AUTO); // TODO: Revert to normal speed FOR EXTEND
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_HANG);

    Actions.runBlocking(rrTrajectories.specimenWallPosToBar2);


    robot.slideExtensionMotor.setPower(1.0); // TODO: FULL SPEED RETRACT
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG_PULL);
    sleep(550);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    sleep(100);

    robot.slideExtensionMotor.setTargetPosition(0);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP_WALL);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_STRAIGHT);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP_WALL);

    // PARK AND RE-SET ZEROS
    Actions.runBlocking(rrTrajectories.barToParkCorner);

    robot.slideRotationMotor.setTargetPosition(0);

    sleep(5000);



  }

  public void pickUpSampleAuto() {
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP);

    sleep(250);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);

    sleep(500);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_DRIVE);

    sleep(250);

  }
}
