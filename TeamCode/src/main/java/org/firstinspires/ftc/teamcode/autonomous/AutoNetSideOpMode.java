package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RRPushTrajectories;
import org.firstinspires.ftc.teamcode.common.RRTrajectories;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Net Zone Autonomous", group = "0Competition")
//@Disabled
public class AutoNetSideOpMode extends LinearOpMode {

  RRTrajectories rrTrajectories;
  Robot robot;

  Action[] trajectories;

  @Override
  public void runOpMode() {
    robot = new Robot(this);
    rrTrajectories = new RRTrajectories(this.hardwareMap);

    robot.initializeArmDevices();
    rrTrajectories.initTrajectories();
    trajectories = rrTrajectories.getLeftSideTrajectories();


    waitForStart();


    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_DROP_OFF_SAMPLES);
    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_STRAIGHT);


    Actions.runBlocking(trajectories[0]);

    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_DROP_TOP_BASKET);

    sleep(1700);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_DROP_DIP);

    sleep(500);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);

    sleep(300);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_STRAIGHT);

    sleep(700);


    robot.slideRotationMotor.setPower(0.3);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_PICKUP - 50);

    Actions.runBlocking(trajectories[1]);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP_DIP);

    sleep(500); // TODO was 300 'bounce fix?'

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);

    sleep(400);

    robot.slideRotationMotor.setPower(1.0);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_DROP_OFF_SAMPLES);
    robot.slideExtensionMotor.setTargetPosition(0);

    Actions.runBlocking(trajectories[2]);

    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_DROP_TOP_BASKET);

    sleep(1700);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_DROP_DIP);

    sleep(500);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);

    sleep(300);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_STRAIGHT);

    sleep(700);

    robot.slideRotationMotor.setPower(0.3);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_PICKUP);
    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_PICKUP);

    Actions.runBlocking(trajectories[3]);

    sleep(500);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_PICKUP_DIP);

    sleep(500); // TODO was 300 'bounce fix?'

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_CLOSED);

    sleep(400);

    robot.slideRotationMotor.setPower(1.0);
    robot.slideRotationMotor.setTargetPosition(Robot.ARM_ROT_AUTO_DROP_OFF_SAMPLES);
    robot.slideExtensionMotor.setTargetPosition(0);

    Actions.runBlocking(trajectories[4]);

    robot.slideExtensionMotor.setTargetPosition(Robot.ARM_EXT_AUTO_DROP_TOP_BASKET);

    sleep(1700);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_DROP_DIP);

    sleep(500);

    robot.setClawGrabServoPosition(Robot.CLAW_GRAB_POSITION_OPEN);

    sleep(300);

    robot.setClawPanServoPosition(Robot.CLAW_PAN_POSITION_AUTO_STRAIGHT);

    sleep(700);

    robot.slideExtensionMotor.setTargetPosition(0);
    sleep(300);
    robot.slideRotationMotor.setPower(0.3);
    robot.slideRotationMotor.setTargetPosition(0);

    sleep(100000);

  }
}
