package org.firstinspires.ftc.teamcode.testing;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.VelocitySupplier;

import java.util.function.DoubleSupplier;

public class HomeostasisTest extends LinearOpMode {
  Robot robot = new Robot(this);
  VelocitySupplier velocitySupplier = new VelocitySupplier();

  double KpVel = 0;
  double KiVel = 0;
  double KdVel = 0;
  double integralSumMaxVel = 0;
  double stabilityThresholdVel = 0;
  double lowPassGainVel = 0;

  PIDCoefficientsEx pidCoefficientsExVel = new PIDCoefficientsEx(KpVel, KiVel, KdVel, integralSumMaxVel, stabilityThresholdVel, lowPassGainVel);
  PIDEx pidExControllerVel = new PIDEx(pidCoefficientsExVel);

  double KpPos = 0;
  double KiPos = 0;
  double KdPos = 0;
  double integralSumMaxPos = 0;
  double stabilityThresholdPos = 0;
  double lowPassGainPos = 0;

  PIDCoefficientsEx pidCoefficientsExPos = new PIDCoefficientsEx(KpPos, KiPos, KdPos, integralSumMaxPos, stabilityThresholdPos, lowPassGainPos);
  PIDEx pidExControllerPos = new PIDEx(pidCoefficientsExPos);

  double Kv = 0;
  double Ka = 0;
  double Ks = 0;
  double Kg = 0; // Keep at zero
  double Kcos = 0;

  FeedforwardCoefficientsEx feedforwardCoefficientsEx = new FeedforwardCoefficientsEx(Kv, Ka, Ks, Kg, Kcos);
  FeedforwardEx feedforwardExController = new FeedforwardEx(feedforwardCoefficientsEx);

  DoubleSupplier motorPosition = () -> velocitySupplier.getPosition();
  DoubleSupplier motorVelocity = () -> velocitySupplier.getVelocity();

  double Q = 0;
  double R = 0;
  int N = 0;

  KalmanEstimator kalmanEstimatorVel = new KalmanEstimator(motorVelocity, Q, R, N);
  KalmanEstimator kalmanEstimatorPos = new KalmanEstimator(motorPosition, Q, R, N);

  PositionVelocitySystem system = new PositionVelocitySystem(
    kalmanEstimatorPos,
    kalmanEstimatorVel,
    feedforwardExController,
    pidExControllerPos,
    pidExControllerVel
  );

  double command = 0;

  @Override
  public void runOpMode() {
    robot.initializeDevices();
    while (opModeIsActive()) {
      velocitySupplier.setPosition(robot.getSlideRotationMotorCurrentPosition());
      command = system.update(velocitySupplier.getPosition(), velocitySupplier.getVelocity(), velocitySupplier.getAcceleration());
    }
  }
}
