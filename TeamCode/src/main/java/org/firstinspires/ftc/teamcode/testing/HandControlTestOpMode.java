package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hand Control Test", group = "Testing")

public class HandControlTestOpMode extends LinearOpMode {
  Servo clawGrabServo = null;

  Gamepad currentGamepad1 = new Gamepad();
  Gamepad prevGamepad1 = new Gamepad();


  double clawClosePosition = 0;

  @Override
  public void runOpMode() {
    clawGrabServo = hardwareMap.get(Servo.class, "clawGrabServo");
    clawGrabServo.setPosition(clawClosePosition);
    waitForStart();

    while (opModeIsActive()) {
      prevGamepad1.copy(currentGamepad1);
      currentGamepad1.copy(gamepad1);
      if (currentGamepad1.right_bumper && !prevGamepad1.right_bumper) {
        if (clawClosePosition == 0) {
          clawClosePosition = 0.175;

        }
        else {
          clawClosePosition = 0;
        }
      }
      clawGrabServo.setPosition(clawClosePosition);
      telemetry.addData("claw position", clawClosePosition);
      telemetry.update();
    }
  }
}
