package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Test", group = "Testing")
public class RevColorSensorV3TestOpMode extends LinearOpMode {

  ColorSensor colorSensor = null;

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

  Color color;

  @Override
  public void runOpMode() {
    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    waitForStart();

    while (opModeIsActive()) {
      red = colorSensor.red();
      green = colorSensor.green();
      blue = colorSensor.blue();
      alpha = colorSensor.alpha();

      if (red > redValues[0][0] && red < redValues[1][0] && green > redValues[0][1] && green < redValues[1][1] && blue > redValues[0][2] && blue < redValues[1][2]){
        color = Color.RED;
      }
      else if (red > yellowValues[0][0] && red < yellowValues[1][0] && green > yellowValues[0][1] && green < yellowValues[1][1] && blue > yellowValues[0][2] && blue < yellowValues[1][2]){
        color = Color.YELLOW;
      }
      else if (red > blueValues[0][0] && red < blueValues[1][0] && green > blueValues[0][1] && green < blueValues[1][1] && blue > blueValues[0][2] && blue < blueValues[1][2]){
        color = Color.BLUE;
      }
      else {
        color = Color.UNKNOWN;
      }
      telemetry.addData("Color", color.toString());
      telemetry.addData("Red", red);
      telemetry.addData("Green", green);
      telemetry.addData("Blue", blue);
      telemetry.addData("Alpha", alpha);
      telemetry.update();
    }
  }
}