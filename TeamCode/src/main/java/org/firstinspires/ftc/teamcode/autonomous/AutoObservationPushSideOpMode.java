package org.firstinspires.ftc.teamcode.autonomous;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.RRPushTrajectories;
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
    robot.slideRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.slideRotationMotor.setTargetPosition(0);
    robot.slideRotationMotor.setPower(1);
    robot.slideRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //robot.slideExtensionMotor.setPower(Robot.ARM_EXT_POWER_AUTO);
    robot.slideExtensionMotor.setPower(1);
    rrTrajectories.initTrajectories();
    trajectories = rrTrajectories.getRightSideTrajectories();

    waitForStart();

    // HANG FIRST (HELD) SPECIMEN
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_HANG);
    // sleep(200);

    Actions.runBlocking(rrTrajectories.rightStartToBar);

    robot.slideExtensionMotor.setPower(1.0); // TODO FULL SPEED RETRACT ON HANG PULL
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

    //double x = robot.getLeftDistance();
    //double y = robot.getRightDistance();

    double Q = 0.3; // High values put more emphasis on the sensor.
    double R = 4; // High Values put more emphasis on regression.
    int N = 3; // The number of estimates in the past we perform regression on.
    KalmanFilter filterLeft = new KalmanFilter(Q,R,N);
    KalmanFilter filterRight = new KalmanFilter(Q,R,N);

    double currentValueL = robot.getLeftDistance();  // noisy sensor
    double estimateL = filterLeft.estimate(currentValueL); // smoothed sensor
    double currentValueR = robot.getRightDistance();  // noisy sensor
    double estimateR = filterRight.estimate(currentValueR); // smoothed sensor

    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    // Get up to 'N' values for linear regression to create prediction TODO: UNTESTED
    for (int i=0; i<10; i++) {
      currentValueL = robot.getLeftDistance();  // imaginary, noisy sensor
      estimateL = filterLeft.estimate(currentValueL); // smoothed sensor
      currentValueR = robot.getRightDistance();  // noisy sensor
      estimateR = filterRight.estimate(currentValueR); // smoothed sensor

      telemetry.addData("init left", String.format("%.01f mm", estimateL));
      telemetry.addData("init right", String.format("%.01f mm", estimateR)); currentValueL = robot.getLeftDistance();  // imaginary, noisy sensor
      telemetry.update();
    }

    // use smoothed value
    while (estimateL > 155) {
      rrTrajectories.drive.rightFront.setPower(Robot.DRIVE_TRAIN_SPEED_AUTO_TO_BAR);
      rrTrajectories.drive.leftFront.setPower(Robot.DRIVE_TRAIN_SPEED_AUTO_TO_BAR);
      rrTrajectories.drive.rightBack.setPower(Robot.DRIVE_TRAIN_SPEED_AUTO_TO_BAR);
      rrTrajectories.drive.leftBack.setPower(Robot.DRIVE_TRAIN_SPEED_AUTO_TO_BAR);

      // get next value
      // x = robot.getLeftDistance();
      // y = robot.getRightDistance();
      currentValueL = robot.getLeftDistance();  // imaginary, noisy sensor
      estimateL = filterLeft.estimate(currentValueL); // smoothed sensor
//      currentValueR = robot.getRightDistance();  // noisy sensor
//      estimateR = filterRight.estimate(currentValueR); // smoothed sensor
      telemetry.addData("only left", String.format("%.01f mm", estimateL));
      telemetry.addData("not used right", String.format("%.01f mm", estimateR));
      telemetry.update();
    }


    rrTrajectories.drive.rightFront.setPower(0);
    rrTrajectories.drive.leftFront.setPower(0);
    rrTrajectories.drive.rightBack.setPower(0);
    rrTrajectories.drive.leftBack.setPower(0);
    robot.slideExtensionMotor.setPower(1.0); // TODO FULL SPEED RETRACT
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG_PULL);
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

    // Get up to 'N' values for linear regression to create prediction TODO: UNTESTED
    for (int i=0; i<10; i++) {
      currentValueL = robot.getLeftDistance();  // imaginary, noisy sensor
      estimateL = filterLeft.estimate(currentValueL); // smoothed sensor
      currentValueR = robot.getRightDistance();  // noisy sensor
      estimateR = filterRight.estimate(currentValueR); // smoothed sensor

      telemetry.addData("init 2 left", String.format("%.01f mm", estimateL));
      telemetry.addData("init 2 right", String.format("%.01f mm", estimateR));
      telemetry.update();

    }


    while ((estimateL + estimateR) / 2 > 210) {
      rrTrajectories.drive.rightFront.setPower(Robot.DRIVE_TRAIN_SPEED_AUTO_TO_BAR);
      rrTrajectories.drive.leftFront.setPower(Robot.DRIVE_TRAIN_SPEED_AUTO_TO_BAR);
      rrTrajectories.drive.rightBack.setPower(Robot.DRIVE_TRAIN_SPEED_AUTO_TO_BAR);
      rrTrajectories.drive.leftBack.setPower(Robot.DRIVE_TRAIN_SPEED_AUTO_TO_BAR);

      currentValueL = robot.getLeftDistance();  // imaginary, noisy sensor
      estimateL = filterLeft.estimate(currentValueL); // smoothed sensor
      currentValueR = robot.getRightDistance();  // noisy sensor
      estimateR = filterRight.estimate(currentValueR); // smoothed sensor
      telemetry.addData("both left", String.format("%.01f mm", estimateL));
      telemetry.addData("both right", String.format("%.01f mm", estimateR));
      telemetry.update();
    }


    rrTrajectories.drive.rightFront.setPower(0);
    rrTrajectories.drive.leftFront.setPower(0);
    rrTrajectories.drive.rightBack.setPower(0);
    rrTrajectories.drive.leftBack.setPower(0);

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
}