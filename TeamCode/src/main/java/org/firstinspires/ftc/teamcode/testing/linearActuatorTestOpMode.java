package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Linear Acutator Test", group = "zTesting")
@Disabled
public class linearActuatorTestOpMode extends LinearOpMode {

  // Initialize Variables

  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor actuatorMotor = null;

  double rStickY;

  @Override
  public void runOpMode() throws InterruptedException {

    telemetry.addData("Status", "Initializing");
    telemetry.update();

    actuatorMotor = hardwareMap.get(DcMotor.class, "actuatorMotor");

    actuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    telemetry.addData("Status", "Initialized");
    telemetry.addLine("This is a very cool telemetry Message");
    telemetry.update();

    waitForStart();

    while (opModeIsActive()) {
      telemetry.addData("Status", "Running");

      rStickY = gamepad2.right_stick_y;
      telemetry.addData("Gamepad2 Inputs", "Right Stick Y: %.2f", rStickY);
      actuatorMotor.setPower(-rStickY);

      telemetry.update();
    }
  }
}