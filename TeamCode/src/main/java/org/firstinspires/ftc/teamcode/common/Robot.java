package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Robot.Color.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {

  // Motors and devices

  public DcMotor frontLeftMotor = null;
  public DcMotor frontRightMotor = null;
  public DcMotor backLeftMotor = null;
  public DcMotor backRightMotor = null;

  public DcMotorEx slideExtensionMotor = null;
  public DcMotorEx slideRotationMotor = null;

  public Servo clawGrabServo = null;
  public Servo clawPanServo = null;
  public Servo clawRotateServo = null;

  LinearOpMode myOpMode;

  // Constants

  public enum Team {
    RED,
    BLUE
  }

  public static double CLAW_GRAB_POSITION_CLOSED = 0.5; // was 0.0 (old finger clocking)
  public static double CLAW_GRAB_POSITION_OPEN = 1.0; // was (0.625 old finger clocking)

  public static double CLAW_ROTATE_POSITION_STRAIGHT = 0.5;
  public static double CLAW_ROTATE_POSITION_AUTO_PICKUP = 0.65;
  public static double CLAW_ROTATE_POSITION_RIGHT = 0.8375;

  public static double CLAW_PAN_TELEOP_INIT = 0.65;
  public static double CLAW_PAN_POSITION_DROP_DIP = 0.55; // don't retract slide with this position!!!
  public static double CLAW_PAN_POSITION_STRAIGHT = 0.205;
  public static double CLAW_PAN_POSITION_PICKUP_DIP = 0.115;
  public static double CLAW_PAN_POSITION_PICKUP_WALL = 0.5450;
  public static double CLAW_PAN_POSITION_AUTO_PICKUP_WALL = 0.5494;
  public static double CLAW_PAN_POSITION_TOP_SPECIMEN = 0.22;
  public static double CLAW_PAN_POSITION_DRIVE = 0.1994;
  public static double CLAW_PAN_POSITION_HANG_ROBOT = 0.075;
  public static double CLAW_PAN_POSITION_AUTO_HANG = 0.2;
  public static double CLAW_PAN_POSITION_AUTO_DROP_DIP = 0.6;
  public static double CLAW_PAN_POSITION_AUTO_STRAIGHT = 0.225;
  public static double CLAW_PAN_POSITION_AUTO_PICKUP = 0.225;
  public static double CLAW_PAN_POSITION_AUTO_PICKUP_DIP = 0.135;
  public static double CLAW_PAN_SPEED = 0.025;

  public static double CLAW_ROTATE_SPEED = 0.100;
  public static double CLAW_ROTATE_MAX = 0.8125;
  public static double CLAW_ROTATE_MIN = 0.1825;

  // Extension constants with 117 RPM motor
//  public static int ARM_EXT_INIT = 0;
//  public static int ARM_EXT_DROP_TOP_BASKET = 8085;
//  public static int ARM_EXT_DROP_BOTTOM_BASKET = 2550;
//  public static int ARM_EXT_HANG_TOP_SPECIMEN = 1450;
//  public static int ARM_EXT_HANG_TOP_SPECIMEN_PULL = 140;
//  public static int ARM_EXT_PICKUP_SAMPLES = 2293;
//  public static int ARM_EXT_DRIVE = 0;
//  public static int ARM_EXT_PICKUP_WALL = 0;
//  public static int ARM_EXT_HANG_ROBOT = 4950;
//  public static int ARM_EXT_HANG_ROBOT_PULL = 1652;
//  public static int ARM_EXT_AUTO_HANG = 2320;
//  public static int ARM_EXT_AUTO_HANG_PULL = 350;
//  public static int ARM_EXT_AUTO_PICKUP = 2000;
//  public static int ARM_EXT_AUTO_DROP_OBSERVE = 4600;

  // Extension constants with 312 RPM motor
  public static int ARM_EXT_INIT = 0;
  public static int ARM_EXT_DROP_TOP_BASKET = 3060;
  public static int ARM_EXT_DROP_BOTTOM_BASKET = 1024;
  public static int ARM_EXT_HANG_TOP_SPECIMEN = 628;
  public static int ARM_EXT_HANG_TOP_SPECIMEN_PULL = 28;
  public static int ARM_EXT_PICKUP_SAMPLES = 0;
  public static int ARM_EXT_PICKUP_SAMPLES_EXT = 1661;
  public static int ARM_EXT_DRIVE = 0;
  public static int ARM_EXT_PICKUP_WALL = 28;
  public static int ARM_EXT_HANG_ROBOT = 2100;
  public static int ARM_EXT_HANG_ROBOT_PULL = 200;

  public static int ARM_EXT_AUTO_HANG = 1100; // TODO: 1009(1:21pm) 24Jan->1034->1040 little higher for different grip on spec
  public static int ARM_EXT_AUTO_HANG_PULL = 425;
  public static int ARM_EXT_AUTO_PICKUP = 1152; //todo fine tune
  public static int ARM_EXT_AUTO_DROP_OBSERVE = (int) (4600/2.66); //todo fine tune
  public static int ARM_EXT_AUTO_DROP_TOP_BASKET = 3060;

  public static int ARM_ROT_INIT = 0;
  public static int ARM_ROT_DROP_OFF_SAMPLES = 1550;
  public static int ARM_ROT_DROP_OFF_SAMPLES_BOTTOM = 1575;
  public static int ARM_ROT_HANG_TOP_SPECIMEN = 1202;
  public static int ARM_ROT_PICKUP_SAMPLES = 300;
  public static int ARM_ROT_PICKUP_WALL = 241;
  public static int ARM_ROT_AUTO_PICKUP_WALL = 297;
  public static int ARM_ROT_DRIVE = 650;
  public static int ARM_ROT_HANG_ROBOT = 1050;
  public static int ARM_ROT_AUTO_HANG = 1160; //1068; //1160
  public static int ARM_ROT_AUTO_DROP_OFF_SAMPLES = 1580;
  public static int ARM_ROT_AUTO_DRIVE = 1123;
  public static int ARM_ROT_AUTO_PICKUP = 356;

  public static double ARM_ROT_POWER = 1.0;  // TODO 0.5 (testing PID issue)
  public static double ARM_ROT_POWER_FULL = 1.0;
  public static double ARM_EXT_POWER = 1.0;
  public static double ARM_EXT_POWER_AUTO = 0.38;
  public static double DRIVE_TRAIN_SPEED_FAST = 1;
  public static double DRIVE_TRAIN_SPEED_SLOW = 1.0 / 3.0;

  public static double LENGTH_CLAW = 7;
  public static double LENGTH_INSPECTION_FRONT = 35;
  public static double LENGTH_INSPECTION_BACK = 100;
  public static double LENGTH_ARM_EXTENDED = 50;
  public static double LENGTH_ARM_NORMAL = 13.375;
  public static double LIMELIGHT_CAMERA_HEIGHT = 12.5;
  public static double LIMELIGHT_CAMERA_ANGLE = 80.0;
  public static double LENGTH_CIRCUMFERENCE_WHEEL = 10.995574287564276;

  public static double CONVERT_DEGREES_TICKS_117RPM = 3.95861111111;
  public static double CONVERT_TICKS_DEGREES_117RPM = 1.0 / CONVERT_DEGREES_TICKS_117RPM;
  public static double CONVERT_DEGREES_TICKS_312RPM = 1.4936111111;
  public static double CONVERT_TICKS_DEGREES_312RPM = 1.0 / CONVERT_DEGREES_TICKS_117RPM;
  public static double CONVERT_DEGREES_INCHES_SLIDE = 0.013177365175032795;
  public static double CONVERT_INCHES_DEGREES_SLIDE = 1.0 / CONVERT_DEGREES_INCHES_SLIDE;

  // Arm down control
  public static int SECONDS_DOWN_FAST = 6000;
  public static int SECONDS_DOWN_SLOW = 8000;
  public static int CYCLE_TIME = 5;

  double rotX;
  double rotY;
  double denominator;
  double frontLeftPower;
  double frontRightPower;
  double backLeftPower;
  double backRightPower;

  boolean prevExtending = false;
  boolean prevRotating = false;

  // max extension
  double maxExtension;

  // arm target position
  int slideRotationTargetPosition = ARM_ROT_DRIVE;
  int slideExtensionTargetPosition = ARM_EXT_INIT;

  // Hand position
  double clawGrabPosition = CLAW_GRAB_POSITION_CLOSED;
  double clawPanPosition = CLAW_PAN_TELEOP_INIT;
  double clawRotatePosition = CLAW_ROTATE_POSITION_STRAIGHT;
  double driveSpeed = DRIVE_TRAIN_SPEED_FAST;

  RevColorSensorV3 colorSensor = null;

  int red;
  int green;
  int blue;
  int alpha;

  int[][] redValues = {{300, 150, 50}, {1400, 800, 500}};
  int[][] yellowValues = {{1000, 1550, 100}, {3000, 4000, 1000}};
  int[][] blueValues = {{50, 100, 800}, {350, 700, 2100}};

  public enum Color {
    RED,
    YELLOW,
    BLUE,
    UNKNOWN
  }

  public Team team = Team.RED;

  Color color;



  public Robot(LinearOpMode opMode) {
    myOpMode = opMode;
  }

  public void initializeDevices() {
    frontLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
    frontRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
    backLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "backRightMotor");

    frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    initializeArmDevices();
  }

  public void initializeArmDevices() {
    slideExtensionMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "slideExtensionMotor");
    slideRotationMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "slideRotationMotor");

    clawGrabServo = myOpMode.hardwareMap.get(Servo.class, "clawGrabServo");
    clawPanServo = myOpMode.hardwareMap.get(Servo.class, "clawPanServo");
    clawRotateServo = myOpMode.hardwareMap.get(Servo.class, "clawRotateServo");

    colorSensor = myOpMode.hardwareMap.get(RevColorSensorV3.class, "colorSensor");

    slideRotationMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    //slideRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //slideRotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    controller = new PIDController(UP_UNEXTENDED_KP, UP_UNEXTENDED_KI, UP_UNEXTENDED_KD);

    slideExtensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    slideExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slideExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slideExtensionMotor.setTargetPosition(ARM_EXT_INIT);
    slideExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slideExtensionMotor.setPower(1);

    clawPanServo.setPosition(CLAW_PAN_TELEOP_INIT);
    clawRotateServo.setPosition(CLAW_ROTATE_POSITION_STRAIGHT);
    clawGrabServo.setPosition(CLAW_GRAB_POSITION_CLOSED);
  }

  public void setMotorPowers(double x, double y, double rx, double heading, double speed) {
    rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
    rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

    // put strafing factors here
    rotX = rotX * 1;
    rotY = rotY * 1;

    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    frontLeftPower = (rotY - rotX - rx) / denominator;
    backLeftPower = (rotY + rotX - rx) / denominator;
    frontRightPower = (rotY + rotX + rx) / denominator;
    backRightPower = (rotY - rotX + rx) / denominator;

    frontLeftMotor.setPower(frontLeftPower * speed);
    backLeftMotor.setPower(backLeftPower * speed);
    frontRightMotor.setPower(frontRightPower * speed);
    backRightMotor.setPower(backRightPower * speed);
  }

  public void setMotorPowers(double x, double y, double rx, double heading) {
    rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
    rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

    // put strafing factors here
    rotX = rotX * 1;
    rotY = rotY * 1;

    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

    frontLeftPower = (rotY - rotX - rx) / denominator;
    backLeftPower = (rotY + rotX - rx) / denominator;
    frontRightPower = (rotY + rotX + rx) / denominator;
    backRightPower = (rotY - rotX + rx) / denominator;

    frontLeftMotor.setPower(frontLeftPower * driveSpeed);
    backLeftMotor.setPower(backLeftPower * driveSpeed);
    frontRightMotor.setPower(frontRightPower * driveSpeed);
    backRightMotor.setPower(backRightPower * driveSpeed);
  }

  // Toggle drive speed
  public void toggleDriveSpeed() {
    driveSpeed = driveSpeed == DRIVE_TRAIN_SPEED_FAST ? DRIVE_TRAIN_SPEED_SLOW : DRIVE_TRAIN_SPEED_FAST;
  }

  // Get drive speed
  public double getDriveSpeed() {
    return driveSpeed;
  }

  // Set Slide Rotation Motor power
  public double slideRotationPower = 1;
  public void setSlideRotationMotorPower(double power) {
    slideRotationPower = power;
  }

  // Set Slide Rotation Motor position
  public void setSlideRotationMotorTargetPosition(int position) {
    slideRotationTargetPosition = position;
  }

  // Get Slide Rotation Motor Target position
  public int getSlideRotationMotorTargetPosition() {
    return slideRotationTargetPosition;
  }

  // Get Slide Rotation Motor Current position
  public int getSlideRotationMotorCurrentPosition() {
    return slideRotationMotor.getCurrentPosition();
  }

  // Set Slide Extension Motor power
  public void setSlideExtensionMotorPower(double power) {
    slideExtensionMotor.setPower(power);
  }

  // Set Slide Extension Motor position
  // set the extension without soft limit, example 0
  public void setSlideExtensionMotorTargetPosition(int position) {
    if (position > ARM_EXT_DROP_TOP_BASKET)
      slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    else if (position < 0)
      slideExtensionTargetPosition = 0;
    else
      slideExtensionTargetPosition = position;
  }

  public void setSlideExtMotorTargetPosWithLimit(int position) {
    checkSoftLimits(convertTicksToDegrees312RPM(slideExtensionMotor.getCurrentPosition()) * Robot.CONVERT_DEGREES_INCHES_SLIDE,
            (armPos - 327) / 14.6697222222);

    slideExtensionTargetPosition = convertDegreesToTicks312RPM((maxExtension - LENGTH_ARM_NORMAL) * CONVERT_INCHES_DEGREES_SLIDE);
  }

  // Get Slide Extension Motor Target position
  public int getSlideExtensionMotorTargetPosition() {
    return slideExtensionMotor.getTargetPosition();
  }

  // Get Slide Extension Motor Current position
  public int getSlideExtensionMotorCurrentPosition() {
    return slideExtensionMotor.getCurrentPosition();
  }

  // Set claw Grab Servo position
  public void setClawGrabServoPosition(double grabPosition) {
    clawGrabServo.setPosition(grabPosition);
  }

  // Toggle Finger position
  public void toggleClawGrabPosition() {
    clawGrabPosition = clawGrabServo.getPosition() == CLAW_GRAB_POSITION_CLOSED ? CLAW_GRAB_POSITION_OPEN : CLAW_GRAB_POSITION_CLOSED;
  }

  // Set claw Pan Servo position
  public void setClawPanServoPosition(double panPosition) {
    clawPanPosition = panPosition;
    clawPanServo.setPosition(panPosition);
  }

  // Move claw wrist up
  public void clawPanServoUp() {
    clawPanPosition = (clawPanServo.getPosition() + CLAW_PAN_SPEED) > 1 ? 1 : (clawPanServo.getPosition() + CLAW_PAN_SPEED);
  }

  // Move claw wrist down
  public void clawPanServoDown() {
    clawPanPosition = (clawPanServo.getPosition() - CLAW_PAN_SPEED) < 0 ? 0 : (clawPanServo.getPosition() - CLAW_PAN_SPEED);
  }

  // Move claw wrist up
  public void clawRotateServoLeft() {
    clawRotatePosition = (clawRotateServo.getPosition() + CLAW_ROTATE_SPEED) > CLAW_ROTATE_MAX ? CLAW_ROTATE_MAX : (clawRotateServo.getPosition() + CLAW_PAN_SPEED);
  }

  // Move claw wrist down
  public void clawRotateServoRight() {
    clawRotatePosition = (clawRotateServo.getPosition() - CLAW_ROTATE_SPEED) < CLAW_ROTATE_MIN ? CLAW_ROTATE_MIN : (clawRotateServo.getPosition() - CLAW_PAN_SPEED);
  }
  //set claw rotate servo position
  public void setClawRotateServoPosition(double RotatePosition) {
    clawRotateServo.setPosition(RotatePosition);
  }

  public void toggleClawRotation() {
    clawRotatePosition = clawRotateServo.getPosition() == CLAW_ROTATE_POSITION_STRAIGHT ? CLAW_ROTATE_POSITION_RIGHT : CLAW_ROTATE_POSITION_STRAIGHT;
  }

  public void pickupPosition() {
    slideRotationPower = ARM_ROT_POWER;
    slideRotationTargetPosition = ARM_ROT_PICKUP_SAMPLES;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_PICKUP_SAMPLES;
    clawPanPosition = CLAW_PAN_POSITION_STRAIGHT;
    clawGrabPosition = CLAW_GRAB_POSITION_OPEN;
  }

  public void dropTopBasket() {
    slideRotationPower = ARM_ROT_POWER;
    slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    clawPanPosition = CLAW_PAN_POSITION_STRAIGHT;
  }

  // Bottom Basket Drop
  public void dropBottomBasket() {
    slideRotationPower = ARM_ROT_POWER;
    slideRotationTargetPosition = ARM_ROT_DROP_OFF_SAMPLES_BOTTOM;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_DROP_BOTTOM_BASKET;
    clawPanPosition = CLAW_PAN_POSITION_STRAIGHT;
  }

  //Wall Pickup
  public void wallPickup () {
    slideRotationPower = ARM_ROT_POWER;
    slideRotationTargetPosition = ARM_ROT_PICKUP_WALL;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_PICKUP_WALL;
    clawPanPosition = CLAW_PAN_POSITION_PICKUP_WALL;
    clawGrabPosition = CLAW_GRAB_POSITION_OPEN;
  }

  // Top Specimen Bar
  public void topSpecimenBar () {
    slideRotationPower = ARM_ROT_POWER;
    slideRotationTargetPosition = ARM_ROT_HANG_TOP_SPECIMEN;
    clawPanPosition = CLAW_PAN_POSITION_TOP_SPECIMEN;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_HANG_TOP_SPECIMEN;
  }

  public void pullExtToHangSpecimen() {
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_HANG_TOP_SPECIMEN_PULL;

  }

  // Bottom Specimen Bar TODO
  public void bottomSpecimenBar () {
  }

  // Drive Position
  public void drivePosition () {
    slideRotationPower = ARM_ROT_POWER;
    slideRotationTargetPosition = ARM_ROT_DRIVE;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_DRIVE;
    clawPanPosition = CLAW_PAN_POSITION_DRIVE;
  }

  // ready to hang on first bar TODO
  public void readyHangFirstBar () {

  }
  // hang on first bar TODO
  public void hangFirstBar () {

  }

  // TODO: check soft limit need to be fixed. drop top basket macro doesn't work with limit
  public void checkExtentionLimit () {
    // check limit
    checkSoftLimits(convertTicksToDegrees312RPM(slideExtensionMotor.getCurrentPosition()) * Robot.CONVERT_DEGREES_INCHES_SLIDE,
            slideRotationMotor.getCurrentPosition() / 14.6697222222 - 17.6);
    // check to see if the
    int max = convertDegreesToTicks312RPM((maxExtension - LENGTH_ARM_NORMAL) * CONVERT_INCHES_DEGREES_SLIDE);
    if (slideExtensionMotor.getCurrentPosition() > max) {
      slideExtensionTargetPosition = max;
    }
    if (slideExtensionTargetPosition > ARM_EXT_DROP_TOP_BASKET)
      slideExtensionTargetPosition = ARM_EXT_DROP_TOP_BASKET;
    else if (slideExtensionTargetPosition < ARM_EXT_INIT)
      slideExtensionTargetPosition = ARM_EXT_INIT;
  }

  // Target position has been set, now move the arm to position
  public void moveArmToPosition() {
    if (slideExtensionTargetPosition > slideExtensionMotor.getCurrentPosition()) {
      moveSlideRotationPIDF(slideRotationTargetPosition, slideRotationPower);
      if (Math.abs(slideRotationTargetPosition - slideRotationMotor.getCurrentPosition()) < 50) {
//        checkExtentionLimit();
        slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);
      }
    } else if (slideExtensionTargetPosition < slideExtensionMotor.getCurrentPosition()) {
      slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);
      if (Math.abs(slideExtensionTargetPosition - slideExtensionMotor.getCurrentPosition()) < 30) {
        moveSlideRotationPIDF(slideRotationTargetPosition, slideRotationPower);
      }
    } else {
      moveSlideRotationPIDF(slideRotationTargetPosition, slideRotationPower);
//      checkExtentionLimit();
      slideExtensionMotor.setTargetPosition(slideExtensionTargetPosition);
    }
  }

  public PIDController controller;

  public static double UP_UNEXTENDED_KP = 0.02,   UP_UNEXTENDED_KI = 0.05, UP_UNEXTENDED_KD = 0.0005;
  public static double DOWN_UNEXTENDED_KP = 0.02, DOWN_UNEXTENDED_KI = 0.01, DOWN_UNEXTENDED_KD = 0.0001;
  public static double UP_EXTENDED_KP = 0.04,     UP_EXTENDED_KI = 0.05, UP_EXTENDED_KD = 0.00;
  public static double DOWN_EXTENDED_KP = 0.04,   DOWN_EXTENDED_KI = 0.01, DOWN_EXTENDED_KD = 0.00;
  public static double UNEXTENDED_KCOS = 0.12;
  public static double EXTENDED_KCOS = 0.35;

  public double Kp = 0.003, Ki = 0.02, Kd = 0.0005;


  public double interpolation;

  private final double ticks_in_degree = 3895.9 / 360;

  int armPos;
  double ffOutput;
  double pidOutput;
  double power;

  public void moveSlideRotationPIDF(double target, double powerMult) {
    armPos = slideRotationMotor.getCurrentPosition();
    //interpolation = 1 - (slideExtensionMotor.getCurrentPosition() / 3060.0);

/*
    Kp = target > armPos ? DOWN_UNEXTENDED_KP * interpolation + DOWN_EXTENDED_KP * (1 - interpolation) : UP_UNEXTENDED_KP * interpolation + UP_EXTENDED_KP * (1 - interpolation);
    Ki = target > armPos ? DOWN_UNEXTENDED_KI * interpolation + DOWN_EXTENDED_KI * (1 - interpolation) : UP_UNEXTENDED_KI * interpolation + UP_EXTENDED_KI * (1 - interpolation);
    Kd = target > armPos ? DOWN_UNEXTENDED_KD * interpolation + DOWN_EXTENDED_KD * (1 - interpolation) : UP_UNEXTENDED_KD * interpolation + UP_EXTENDED_KD * (1 - interpolation);

 */

    // Kp = Kp * powerMult; // TODO this didn't work

    controller.setPID(Kp, Ki, Kd);

    pidOutput = controller.calculate(armPos, target);

    ffOutput = Math.cos(Math.toRadians(slideRotationMotor.getCurrentPosition() / ticks_in_degree - 17)) * UNEXTENDED_KCOS;

    // power = Math.max(Math.min(pidOutput, 1), - 2 * (1 - ffOutput)) * powerMult + ffOutput;      // todo TRY a partial scalar
    power = pidOutput*Math.min((Math.abs(powerMult)+0.0),1)+ffOutput; // todo kinda works but inverse dip at lower powers
    // power = pidOutput + ffOutput;


    slideRotationMotor.setPower(power);
    myOpMode.telemetry.addData("Kp", "%f", Kp);
    myOpMode.telemetry.addData("Ki", "%f", Ki);
    myOpMode.telemetry.addData("Kd", "%f", Kd);
    myOpMode.telemetry.addData("pidOutput", "%f", pidOutput);
    myOpMode.telemetry.addData("ffoutput", "%f", ffOutput);
    myOpMode.telemetry.addData("power mult", powerMult);
  }

  public boolean armReachedTarget() {
    if (Math.abs(slideExtensionTargetPosition - slideExtensionMotor.getCurrentPosition()) < 40
            && Math.abs(slideRotationTargetPosition - slideRotationMotor.getCurrentPosition()) < 40)
      return true;
    else
      return false;
  }
  public void moveArmToDropTop() {
    moveArmToPosition();

  }
  public void moveHandToPosition() {
    clawGrabServo.setPosition(clawGrabPosition);
    clawPanServo.setPosition(clawPanPosition);
    clawRotateServo.setPosition(clawRotatePosition);
  }

  public void handPickUpdip() {
    clawPanPosition = CLAW_PAN_POSITION_PICKUP_DIP;
    clawPanServo.setPosition(clawPanPosition);
  }

  public boolean isFingersOpen() {
    return (clawGrabServo.getPosition() == CLAW_GRAB_POSITION_OPEN);
  }
  public void closeFingers() {
    clawGrabPosition = CLAW_GRAB_POSITION_CLOSED;
    clawGrabServo.setPosition(clawGrabPosition);
  }
  public void openFingers() {
    clawGrabPosition = CLAW_GRAB_POSITION_OPEN;
    clawGrabServo.setPosition(clawGrabPosition);
  }
  public void handStraight() {
    clawPanPosition = CLAW_PAN_POSITION_STRAIGHT;
    clawPanServo.setPosition(clawPanPosition);
  }

  public void handDropTopDip () {
    clawPanPosition = CLAW_PAN_POSITION_DROP_DIP;
    clawPanServo.setPosition(clawPanPosition);

  }

  // check the soft limit for the arm to stay within the 42" length requirement
  public void checkSoftLimits(double armExtension, double armRotation) {
    if (armRotation <= 90) {
      maxExtension = Math.min((LENGTH_INSPECTION_FRONT - LENGTH_CLAW) / Math.cos(Math.toRadians(armRotation)), LENGTH_ARM_EXTENDED);
    } else {
      maxExtension = Math.min((LENGTH_INSPECTION_BACK - LENGTH_CLAW) / Math.cos(Math.toRadians(180 - armRotation)), LENGTH_ARM_EXTENDED);
    }
    myOpMode.telemetry.addData("Arm Rotation degree", armRotation);
    myOpMode.telemetry.addData("Max Extension Soft Limit", maxExtension);
  }

  public int convertDegreesToTicks117RPM(double degrees) {
    return (int) Math.round(degrees * CONVERT_DEGREES_TICKS_117RPM);
  }

  public double convertTicksToDegrees117RPM(int ticks) {
    return ticks * CONVERT_TICKS_DEGREES_117RPM;
  }

  public int convertDegreesToTicks312RPM(double degrees) {
    return (int) Math.round(degrees * CONVERT_DEGREES_TICKS_312RPM);
  }

  public double convertTicksToDegrees312RPM(int ticks) {
    return ticks * CONVERT_TICKS_DEGREES_312RPM;
  }
  public void getReadyToHangRobot() {
    slideRotationPower = ARM_ROT_POWER;
    slideRotationTargetPosition = ARM_ROT_HANG_ROBOT;
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_HANG_ROBOT;
    clawPanPosition = CLAW_PAN_POSITION_HANG_ROBOT;
  }

  public void HangRobot() {
    slideExtensionMotor.setPower(ARM_EXT_POWER);
    slideExtensionTargetPosition = ARM_EXT_HANG_ROBOT_PULL;
  }

  public boolean isArmPickup() {
    return (Math.abs(slideRotationMotor.getCurrentPosition() - ARM_ROT_PICKUP_SAMPLES) < 130) &&
            ((slideExtensionMotor.getCurrentPosition() > ARM_EXT_PICKUP_SAMPLES) &&
                    (slideExtensionMotor.getCurrentPosition() < (ARM_EXT_PICKUP_SAMPLES_EXT + 50))) &&
            (Math.abs(clawPanServo.getPosition() - CLAW_PAN_POSITION_STRAIGHT) <= 0.1);
  }

  public boolean isArmTopDropReady() {
    return (Math.abs(slideRotationMotor.getCurrentPosition() - ARM_ROT_DROP_OFF_SAMPLES) < 100) &&
            (Math.abs(slideExtensionMotor.getCurrentPosition() - ARM_EXT_DROP_TOP_BASKET) < 500) &&
            (Math.abs(clawPanServo.getPosition() - CLAW_PAN_POSITION_DROP_DIP) <= 0.1);
  }

  public boolean isArmBottomDropReady() {
    return (Math.abs(slideRotationMotor.getCurrentPosition() - ARM_ROT_DROP_OFF_SAMPLES_BOTTOM) < 100) &&
            (Math.abs(slideExtensionMotor.getCurrentPosition() - ARM_EXT_DROP_BOTTOM_BASKET) < 500) &&
            (Math.abs(clawPanServo.getPosition() - CLAW_PAN_POSITION_DROP_DIP) <= 0.1);
  }

  public void setRotationTargetForPickUp() {
    double a = (105.0/1661.0);
    double rotationOffSet = a * slideExtensionMotor.getCurrentPosition();
    slideRotationTargetPosition = (int)(rotationOffSet) + ARM_ROT_PICKUP_SAMPLES;
  }

  // correct pickup height
  public void setPickUpHeight (){
    Color sensedColor = readColor();
    double distance = colorSensor.getDistance(DistanceUnit.INCH);
    myOpMode.telemetry.addData("Color", color.toString());
    myOpMode.telemetry.addData("distance","%f",distance);
    if ( sensedColor == RED || sensedColor == BLUE || sensedColor == YELLOW ) {
      //slideRotationTargetPosition +=
    }
    else {

    }
  }

  public Color readColor () {
    red = colorSensor.red();
    green = colorSensor.green();
    blue = colorSensor.blue();
    alpha = colorSensor.alpha();

    if (red > redValues[0][0] && red < redValues[1][0] && green > redValues[0][1] && green < redValues[1][1] && blue > redValues[0][2] && blue < redValues[1][2]){
      color = RED;
    }
    else if (red > yellowValues[0][0] && red < yellowValues[1][0] && green > yellowValues[0][1] && green < yellowValues[1][1] && blue > yellowValues[0][2] && blue < yellowValues[1][2]){
      color = YELLOW;
    }
    else if (red > blueValues[0][0] && red < blueValues[1][0] && green > blueValues[0][1] && green < blueValues[1][1] && blue > blueValues[0][2] && blue < blueValues[1][2]){
      color = BLUE;
    }
    else {
      color = UNKNOWN;
    }
    return color;
  }

//  public boolean sampleToPickUp() {
//    red = colorSensor.red();
//    green = colorSensor.green();
//    blue = colorSensor.blue();
//    alpha = colorSensor.alpha();
//
//    if (red > redValues[0][0] && red < redValues[1][0] && green > redValues[0][1] && green < redValues[1][1] && blue > redValues[0][2] && blue < redValues[1][2]){
//      color = RED;
//    }
//    else if (red > yellowValues[0][0] && red < yellowValues[1][0] && green > yellowValues[0][1] && green < yellowValues[1][1] && blue > yellowValues[0][2] && blue < yellowValues[1][2]){
//      color = YELLOW;
//    }
//    else if (red > blueValues[0][0] && red < blueValues[1][0] && green > blueValues[0][1] && green < blueValues[1][1] && blue > blueValues[0][2] && blue < blueValues[1][2]){
//      color = BLUE;
//    }
//    else {
//      color = UNKNOWN;
//    }
//
//    myOpMode.telemetry.addData("Color", color.toString());
//    myOpMode.telemetry.addData("Red", red);
//    myOpMode.telemetry.addData("Green", green);
//    myOpMode.telemetry.addData("Blue", blue);
//    myOpMode.telemetry.addData("Alpha", alpha);
//
//    if (color == YELLOW) {
//      return true;
//    }
//    else if (color == RED && team == Team.RED) {
//      return true;
//    }
//    else if (color == BLUE && team == Team.BLUE) {
//      return true;
//    }
//    else {
//      return false;
//    }
//  }
}

