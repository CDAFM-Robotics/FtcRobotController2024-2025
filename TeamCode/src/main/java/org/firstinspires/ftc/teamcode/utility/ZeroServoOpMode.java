package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name = "Test Servo", group = "utility")
public class ZeroServoOpMode extends LinearOpMode {
  Servo servo = null;
  Robot robot = new Robot(this);
  double servoPosition = 0;
  public Gamepad currentGamepad1 = new Gamepad();
  public Gamepad previousGamepad1 = new Gamepad();
  public Gamepad currentGamepad2 = new Gamepad();
  public Gamepad previousGamepad2 = new Gamepad();
  @Override
  public void runOpMode() throws InterruptedException {


    servo = hardwareMap.get(Servo.class, "clawGrabServo");

    servo.setPosition(0);

    waitForStart();

    while (opModeIsActive()) {

      previousGamepad1.copy(currentGamepad1);
      currentGamepad1.copy(gamepad1);
      previousGamepad2.copy(currentGamepad2);
      currentGamepad2.copy(gamepad2);


      if (currentGamepad2.a && !previousGamepad2.a) {
        servoPosition += 0.025;
        servo.setPosition(servoPosition);
      }
      if (currentGamepad2.b && !previousGamepad2.b) {
        servoPosition -= 0.025;
        servo.setPosition(servoPosition);
      }
      telemetry.addData("Position", servoPosition);
      telemetry.update();
    }
  }
}
