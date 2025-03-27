package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ARMCONTROL", group = "testing")
public class newArmControlTestOpMode extends LinearOpMode{
    Robot robot = new Robot(this);
    DcMotor linearActuatorRight = null;
    DcMotor linearActuatorLeft = null;
    DcMotor slideRotationMotor = null;
    DcMotor slideExtensionMotor = null;

    Servo clawGrabServo = null;
    Servo clawPanServo = null;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void runOpMode() {

        linearActuatorRight = hardwareMap.get(DcMotor.class, "slideExtensionMotorRight");
        linearActuatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearActuatorLeft = hardwareMap.get(DcMotor.class, "slideExtensionMotorLeft");
        linearActuatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearActuatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRotationMotor = hardwareMap.get(DcMotor.class, "slideRotationMotor");
        slideRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slideRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideExtensionMotor = hardwareMap.get(DcMotor.class, "slideUpMotor");
        slideExtensionMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        slideExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtensionMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));

        clawGrabServo = hardwareMap.get(Servo.class, "clawGrabServo");
        clawGrabServo.setPosition(Robot.CLAW_GRAB_POSITION_OPEN);

        clawPanServo = hardwareMap.get(Servo.class, "clawPanServo");

        boolean open = true;
        waitForStart();


        while(opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            //claw grab servo
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper ){
                if (clawGrabServo.getPosition() == Robot.CLAW_GRAB_POSITION_OPEN) {
                    clawGrabServo.setPosition(Robot.CLAW_GRAB_POSITION_CLOSED);

                }
                else {
                    clawGrabServo.setPosition(Robot.CLAW_GRAB_POSITION_OPEN);
                }
            }

            //claw pan servo
            if(currentGamepad1.a && !previousGamepad1.a){
                clawPanServo.setPosition(1);
            }
            if(currentGamepad1.x && !previousGamepad1.x){
                clawPanServo.setPosition(0);
            }

            //arm rotation motor
            if(currentGamepad1.right_trigger >= 0.50 && !(previousGamepad1.right_trigger >= 0.50)){
                if(slideRotationMotor.getTargetPosition() == 0) {
                    slideRotationMotor.setPower(0.125);
                    slideRotationMotor.setTargetPosition(800);
                }
                else{
                    slideRotationMotor.setPower(0.125);
                    slideRotationMotor.setTargetPosition(0);
                }
            }


            //slide extension motor
            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up && slideExtensionMotor.getTargetPosition() != -27000){
                slideExtensionMotor.setPower(3);
                slideExtensionMotor.setTargetPosition(-27000);
            }
            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down && slideExtensionMotor.getTargetPosition() != 0){
                slideExtensionMotor.setPower(3);
                slideExtensionMotor.setTargetPosition(0);
            }

            telemetry.addData("slide rotation motor current position", slideRotationMotor.getCurrentPosition());
            telemetry.addData("slide rotation motor target position", slideRotationMotor.getTargetPosition());
            telemetry.addData("slide extension motor current position", slideExtensionMotor.getCurrentPosition() );
            telemetry.addData("slide extension motor target position", slideExtensionMotor.getTargetPosition() );
            telemetry.addData("claw pan servo", clawPanServo.getPosition());
            telemetry.update();
        }
    }
}
