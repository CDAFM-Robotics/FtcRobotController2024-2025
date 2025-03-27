package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp (name = "AICAMERA", group = "testing")
@Disabled
public class AICameraTestOpMode extends LinearOpMode {

    Robot robot = new Robot(this);
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad currentGamepad2 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();


    private Limelight3A limelight;
    private IMU imu;

    int pipeline = 0;



    @Override
    public void runOpMode(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
        waitForStart();

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(pipeline);


        limelight.start();

        while(opModeIsActive()){

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult result = limelight.getLatestResult();

            telemetry.addData("pipeline", result.getPipelineIndex());

            if (result != null){
                if(result.isValid()){
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.addData("target x", result.getTx());
                    telemetry.addData("target y", result.getTy());
                    telemetry.addData("target Area", result.getTa());
                }
            }else{
                telemetry.addData("Limelight ", "no targets");
            }

            if(currentGamepad2.a && !previousGamepad2.a){

                limelight.pipelineSwitch(pipeline += 1);
            }

            if(pipeline == 10){
                pipeline = 0;
                limelight.pipelineSwitch(pipeline);
            }

            double ydist = Robot.LIMELIGHT_CAMERA_HEIGHT * Math.tan(Math.toRadians(Robot.LIMELIGHT_CAMERA_ANGLE - result.getTx()));
            double xdist = ydist * Math.tan(Math.toRadians(result.getTy()));
            telemetry.addData("result x", xdist);
            telemetry.addData("result y", ydist);

            double rotationsWheelDriveForward = ydist / Robot.LENGTH_CIRCUMFERENCE_WHEEL;
            double rotationsWheelDriveLeft = xdist / Robot.LENGTH_CIRCUMFERENCE_WHEEL;
            telemetry.addData("rotations needed to drive forward", rotationsWheelDriveForward);
            telemetry.addData("rotations needed to drive left", rotationsWheelDriveLeft);

            telemetry.addData("pipeline", pipeline);

            telemetry.update();

            }
        }
    }

