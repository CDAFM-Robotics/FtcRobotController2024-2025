package org.firstinspires.ftc.teamcode.autonomous;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.RRPushTrajectories;
import org.firstinspires.ftc.teamcode.common.Robot;

import java.util.Arrays;
import android.util.Log;

@Autonomous(name = "Observation Zone Push Autonomous", group = "Testing")
public class AutoObservationPushSideOpMode extends LinearOpMode {

  RRPushTrajectories rrTrajectories;
  Robot robot;

  Action[] trajectories;

  @Override
  public void runOpMode() {
    robot = new Robot(this);
    rrTrajectories = new RRPushTrajectories(this.hardwareMap);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


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

    runBlocking(rrTrajectories.rightStartToBar);

    robot.slideExtensionMotor.setPower(1.0); // TODO FULL SPEED RETRACT ON HANG PULL
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG_PULL);
    sleep(550);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    sleep(100);


    robot.slideExtensionMotor.setTargetPosition(0);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP_WALL);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_STRAIGHT);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP_WALL);

    // GO TO Wall 1 (MODIFIED TRAJECTORY PREDynamic)
    runBlocking(rrTrajectories.barToObservationZoneAnd3Samples);

    double currentValueL = robot.getLeftDistance();  // noisy sensor
    double currentValueR = robot.getRightDistance();  // noisy sensor
    Log.w("ROBOT" , "CurrentVal: " + currentValueL + " " + currentValueR );

    // TODO new Dynamic Wall Approach "4.7" inches away from wall should be good (but moves into wall too far) so using 5.5 works
    // TODO FROZEN
    double new_yloc = -55 - (((currentValueL+currentValueR)/2)-5.5);
    Log.w( "ROBOT" , "new_yloc" + new_yloc);

    Action forwardToWallDynamic;
    forwardToWallDynamic = rrTrajectories.drive.actionBuilder(new Pose2d(48, -55, -Math.PI/2))
      .strafeToLinearHeading(new Vector2d(48, new_yloc), -Math.PI/2, new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10), new AngularVelConstraint(Math.PI * 2 / 3))))
      .build();

    runBlocking(forwardToWallDynamic);

    // PICK WALL1
    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);
    sleep(400);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG + 100);
    robot.slideExtensionMotor.setPower(Robot.ARM_EXT_POWER_AUTO);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_HANG);

    // TODO DYNAMIC specimenWallPosToBar (spec1);
    Action specimenWallPosToBarDynamic;
    specimenWallPosToBarDynamic = rrTrajectories.drive.actionBuilder(new Pose2d(48, new_yloc, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(5, -47.5), Math.PI / 2) // was -38 was 38.2
      .build();

    runBlocking(specimenWallPosToBarDynamic);

    currentValueL = robot.getLeftDistance();  // imaginary, noisy sensor
    currentValueR = robot.getRightDistance();  // noisy sensor

    Log.w("ROBOT" , "CurrentVal: " + currentValueL + " " + currentValueR );

    // TODO Measure HOW FAR TO BAR
    new_yloc = -47.5 + (((currentValueL+currentValueR)/2)-3.4);
    Log.w( "ROBOT" , "new_yloc" + new_yloc);

    Action specimenBarApproach1Dynamic;
    specimenBarApproach1Dynamic = rrTrajectories.drive.actionBuilder(new Pose2d(5, -47.5, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(5, new_yloc), Math.PI / 2)
      .build();

    runBlocking(specimenBarApproach1Dynamic);

    // PLACE BAR1
    robot.slideExtensionMotor.setPower(1.0); // TODO FULL SPEED RETRACT
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG_PULL);
    sleep(550);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);
    sleep(100);

    robot.slideExtensionMotor.setTargetPosition(0);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP_WALL);
    robot.setClawRotateServoPosition(Robot.CLAW_ROTATE_POSITION_STRAIGHT);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP_WALL);


    // TODO DYNAMIC BAR TO WALL (Spec2)
    Action barToSpecimenWallPosDynamic = rrTrajectories.drive.actionBuilder(new Pose2d(5, new_yloc, Math.PI / 2))
      .setTangent(-Math.PI / 2)
      .splineToSplineHeading(new Pose2d(48, -55, -Math.PI / 2), 0)
      .build();

    runBlocking(barToSpecimenWallPosDynamic);

    // Find the Wall
    currentValueL = robot.getLeftDistance();
    currentValueR = robot.getRightDistance();

    // TODO Dynamic Wall Approach (spec2)
    new_yloc = -55 - (((currentValueL+currentValueR)/2)-5.5);
    Log.w( "ROBOT" , "new_yloc" + new_yloc);

    Action forwardToWallDynamic2;
    forwardToWallDynamic2 = rrTrajectories.drive.actionBuilder(new Pose2d(48, -55, -Math.PI/2))
      .strafeToLinearHeading(new Vector2d(48, new_yloc), -Math.PI/2, new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10), new AngularVelConstraint(Math.PI * 2 / 3))))
      .build();

    runBlocking(forwardToWallDynamic2);

    // PICK 2
    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);
    sleep(400);

    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_HANG + 100);
    robot.slideExtensionMotor.setPower(Robot.ARM_EXT_POWER_AUTO);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_HANG);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_HANG);

    // TODO Dynamic Wall to Bar (Spec 2)
    Action specimenWallPosToBar2Dynamic = rrTrajectories.drive.actionBuilder(new Pose2d(48, new_yloc, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(3, -47.5), Math.PI / 2)
      .build();

    runBlocking(specimenWallPosToBar2Dynamic);

    currentValueL = robot.getLeftDistance();  // imaginary, noisy sensor
    currentValueR = robot.getRightDistance();  // noisy sensor
    Log.w("ROBOT" , "CurrentVal: " + currentValueL + " " + currentValueR );

    new_yloc = -47.5 + (((currentValueL+currentValueR)/2)-3.4);
    Log.w( "ROBOT" , "new_yloc" + new_yloc);

    // TODO Dynamic Bar Approach 2
    Action specimenBarApproach2Dynamic;
    specimenBarApproach2Dynamic = rrTrajectories.drive.actionBuilder(new Pose2d(3, -47.5, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(3, new_yloc), Math.PI / 2)
      .build();

    runBlocking(specimenBarApproach2Dynamic);

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
    runBlocking(rrTrajectories.barToParkCorner);
    robot.slideRotationMotor.setTargetPosition(0);


    while (currentValueL < 100000) {
      currentValueL = robot.getLeftDistance();  // imaginary, noisy sensor
      currentValueR = robot.getRightDistance();  // noisy sensor
      Log.w("ROBOT" , "CurrentVal: " + currentValueL + " " + currentValueR );
      sleep(1000);
    }




    sleep(5000);
  }
}