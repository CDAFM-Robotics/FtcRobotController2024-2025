package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")
@Config
public class DistanceSensorTestOpMode extends LinearOpMode {

  DistanceSensor distanceSensorLeft;
  DistanceSensor distanceSensorRight;

  @SuppressLint("DefaultLocale")
  @Override
  public void runOpMode() {
    distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
    distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");

    double Q = 0.3; // High values put more emphasis on the sensor.
    double R = 5; // High Values put more emphasis on regression.
    int N = 10; // The number of estimates in the past we perform regression on.
    KalmanFilter filterLeft = new KalmanFilter(Q,R,N);
    KalmanFilter filterRight = new KalmanFilter(Q,R,N);



    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    waitForStart();
    while(opModeIsActive()) {

      double currentValueL = distanceSensorLeft.getDistance(DistanceUnit.MM);  // imaginary, noisy sensor
      double estimateL = filterLeft.estimate(currentValueL); // smoothed sensor

      double currentValueR = distanceSensorLeft.getDistance(DistanceUnit.MM);  // imaginary, noisy sensor
      double estimateR = filterLeft.estimate(currentValueR); // smoothed sensor


      telemetry.addLine("Color Sensor Left:");
      telemetry.addData("deviceName", distanceSensorLeft.getDeviceName());
      telemetry.addData("range left", String.format("%.01f mm", estimateL));

      telemetry.addLine("");

      telemetry.addLine("Color Sensor Right:");
      telemetry.addData("range right", String.format("%.01f mm", /*distanceSensorRight.getDistance(DistanceUnit.MM)*/ estimateR));
      telemetry.update();
    }
  }
}
