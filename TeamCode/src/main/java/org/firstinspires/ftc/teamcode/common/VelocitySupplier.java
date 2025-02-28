package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocitySupplier {
  ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

  double position = 0;
  double velocity = 0;
  double acceleration = 0;

  double previousTime = 0;
  double time = 0;
  double previousPosition = 0;
  double previousVelocity = 0;

  public double getAcceleration() {
    return acceleration;
  }

  public double getVelocity() {
    return velocity;
  }

  public double getPosition() {
    return position;
  }

  public double getPositionRadians() {
    return Math.toRadians(position / 7.7394444444);
  }

  public void setPosition(double position) {
    this.position = position;
    calculateVelocityAcceleration();
  }

  public void calculateVelocityAcceleration() {
    time = elapsedTime.milliseconds() / 1000;
    previousVelocity = velocity;
    velocity = Math.abs(position - previousPosition) / Math.abs(time - previousTime);
    acceleration = Math.abs(velocity - previousVelocity) / Math.abs(time - previousTime);
    previousTime = time;
    previousPosition = position;
  }
}
