package org.firstinspires.ftc.teamcode.testing;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.Estimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.VelocitySupplier;

import java.util.function.DoubleSupplier;

@Config
@TeleOp(name = "Homeostasis Test", group = "Testing")
public class HomeostasisTest extends LinearOpMode {

  DcMotorEx slideRotationMotor = null;
  VelocitySupplier velocitySupplier = new VelocitySupplier();
  /*
  double KpVel = 0;
  double KiVel = 0;
  double KdVel = 0;
  double integralSumMaxVel = 0;
  double stabilityThresholdVel = 0;
  double lowPassGainVel = 0;

  PIDCoefficientsEx pidCoefficientsExVel = new PIDCoefficientsEx(KpVel, KiVel, KdVel, integralSumMaxVel, stabilityThresholdVel, lowPassGainVel);
  PIDEx pidExVel = new PIDEx(pidCoefficientsExVel);
  AngleController pidExControllerVel = new AngleController(pidExVel);

   */

  static double KpPos = 0;
  static double KiPos = 0;
  static double KdPos = 0;
  static double integralSumMaxPos = 0;
  static double stabilityThresholdPos = 0;
  static double lowPassGainPos = 0;

  PIDCoefficientsEx pidCoefficientsExPos = new PIDCoefficientsEx(KpPos, KiPos, KdPos, integralSumMaxPos, stabilityThresholdPos, lowPassGainPos);
  PIDEx pidExPos = new PIDEx(pidCoefficientsExPos);
  AngleController pidExControllerPos = new AngleController(pidExPos);

  static double Kv = 0;
  static double Ka = 0;
  static double Ks = 0;
  static double Kg = 0; // Keep at zero
  static double Kcos = 0;

  FeedforwardCoefficientsEx feedforwardCoefficientsEx = new FeedforwardCoefficientsEx(Kv, Ka, Ks, Kg, Kcos);
  FeedforwardEx feedforwardExController = new FeedforwardEx(feedforwardCoefficientsEx);

  DoubleSupplier motorPosition = () -> slideRotationMotor.getCurrentPosition();
  DoubleSupplier motorVelocity = () -> slideRotationMotor.getVelocity();

  static double Q = 0;
  static double R = 0;
  static int N = 0;

//  KalmanEstimator kalmanEstimatorVel = new KalmanEstimator(motorVelocity, Q, R, N);
  KalmanEstimator kalmanEstimatorPos = new KalmanEstimator(motorPosition, Q, R, N);
  /*
  PositionVelocitySystem system = new PositionVelocitySystem(
    kalmanEstimatorPos,
    kalmanEstimatorVel,
    feedforwardExController,
    pidExControllerPos,
    pidExControllerVel
  );

   */

  BasicSystem basicSystem = new BasicSystem(
    kalmanEstimatorPos,
    pidExControllerPos,
    feedforwardExController
  );

  double command = 0;

  double targetPosition = 0;

  @Override
  public void runOpMode() {

    slideRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    slideRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideRotationMotor.setTargetPosition(0);
    slideRotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    slideRotationMotor.setPower(1);

    while (opModeIsActive()) {
      if (gamepad1.a) {
        targetPosition = 1000;
      }
      if (gamepad1.b) {
        targetPosition = 0;
      }

      command = basicSystem.update(Math.toRadians(targetPosition / 10.82194444444444 - 17.2));
      slideRotationMotor.setPower(command);
    }
  }
}