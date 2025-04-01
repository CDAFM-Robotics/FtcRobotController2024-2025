package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;

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

    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorRight;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "distanceSensorLeft");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "distanceSensorRight");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("Color Sensor Left:");
            telemetry.addData("deviceName", distanceSensorLeft.getDeviceName());
            telemetry.addData("range left", String.format("%.01f mm", distanceSensorLeft.getDistance(DistanceUnit.MM)));

            telemetry.addLine("");

            telemetry.addLine("Color Sensor Right:");
            telemetry.addData("range right", String.format("%.01f mm", distanceSensorRight.getDistance(DistanceUnit.MM)));
            telemetry.update();
        }
    }

}
