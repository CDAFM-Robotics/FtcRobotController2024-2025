package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Robot;

@Config
@TeleOp(name = "PIDF Loop Test", group = "Testing")
public class PIDFLoopTest extends OpMode {

  public PIDController controller;

  public static double UP_UNEXTENDED_KP = 0.02, UP_UNEXTENDED_KI = 0.05, UP_UNEXTENDED_KD = 0.0005;
  public static double DOWN_UNEXTENDED_KP = 0.02, DOWN_UNEXTENDED_KI = 0.01, DOWN_UNEXTENDED_KD = 0.001;
  public static double UP_EXTENDED_KP = 0.04, UP_EXTENDED_KI = 0.05, UP_EXTENDED_KD = 0.0005;
  public static double DOWN_EXTENDED_KP = 0.04, DOWN_EXTENDED_KI = 0.01, DOWN_EXTENDED_KD = 0.001;
  public static double UNEXTENDED_KCOS = 0.12;
  public static double EXTENDED_KCOS = 0.35;

  public double Kp = 0.02, Ki = 0.05, Kd = 0.0005;

  public double interpolation;

  public static int target = 0;

  private final double ticks_in_degree = 3895.9 / 360;

  int armPos;
  double ffOutput;
  double pidOutput;
  double power;

  private DcMotorEx slideRotationMotor;
  private DcMotorEx slideExtensionMotor;
  @Override
  public void init() {
    controller = new PIDController(UP_UNEXTENDED_KP, UP_UNEXTENDED_KI, UP_UNEXTENDED_KD);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    slideRotationMotor = hardwareMap.get(DcMotorEx.class, "slideRotationMotor");
    slideRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    slideExtensionMotor = hardwareMap.get(DcMotorEx.class, "slideExtensionMotor");
    slideExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideExtensionMotor.setTargetPosition(0);
    slideExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


  }

  @Override
  public void loop() {


    armPos = slideRotationMotor.getCurrentPosition();

    interpolation = slideExtensionMotor.getCurrentPosition() / 3060.0;

    Kp = target > armPos ? DOWN_UNEXTENDED_KP * interpolation + DOWN_EXTENDED_KP * (1 - interpolation) : UP_UNEXTENDED_KP * interpolation + UP_EXTENDED_KP * (1 - interpolation);
    Ki = target > armPos ? DOWN_UNEXTENDED_KI * interpolation + DOWN_EXTENDED_KI * (1 - interpolation) : UP_UNEXTENDED_KI * interpolation + UP_EXTENDED_KI * (1 - interpolation);
    Kd = target > armPos ? DOWN_UNEXTENDED_KD * interpolation + DOWN_EXTENDED_KD * (1 - interpolation) : UP_UNEXTENDED_KD * interpolation + UP_EXTENDED_KD * (1 - interpolation);

    controller.setPID(Kp, Ki, Kd);

    pidOutput = controller.calculate(armPos, target);

    slideExtensionMotor.setTargetPosition(3060);
    slideExtensionMotor.setPower(1);

    ffOutput = Math.cos(Math.toRadians(target / ticks_in_degree - 17)) * UNEXTENDED_KCOS * interpolation + EXTENDED_KCOS * (1 - interpolation);

    power = ffOutput + pidOutput;

    slideRotationMotor.setPower(power);
  }
}
