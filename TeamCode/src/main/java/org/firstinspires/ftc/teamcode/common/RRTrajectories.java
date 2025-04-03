package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class RRTrajectories {

  HardwareMap myHardwareMap;

  MecanumDrive drive;

  // Observation Side

  public Action rightStartToBar;
  public Action barToFirstAllySample;
  public Action firstAllySampleToObservationZone;
  public Action observationZoneToSecondAllySample;
  public Action secondAllySampleToObservationZone;
  public Action observationZoneToThirdAllySample;
  public Action thirdAllySampleToObservationZone;
  public Action observationZoneToSpecimenWallPos;
  public Action specimenWallPosToBar;
  public Action barToSpecimenWallPos;
  public Action specimenWallPosToBar2;
  public Action barToSpecimenWallPos2;
  public Action specimenWallPosToBar3;
  public Action barToSpecimenWallPos3;
  public Action barToParkCorner;


  public Action[] rightSideTrajectories;

  // Sample Side

  public Action leftStartToNet;
  public Action netToFirstYellowSample;
  public Action firstYellowSampleToNet;
  public Action netToSecondYellowSample;
  public Action secondYellowSampleToNet;
  public Action netToThirdYellowSampleWall;
  public Action thirdYellowSampleWallToNet;

  public Action[] leftSideTrajectories;

  public RRTrajectories(HardwareMap hardwareMap) {
    myHardwareMap = hardwareMap;
  }
  public void initTrajectories() {

    drive = new MecanumDrive(myHardwareMap, new Pose2d(24, -65 ,Math.PI / 2));

    rightStartToBar = drive.actionBuilder(new Pose2d(24, -65, Math.PI / 2))
      .splineToConstantHeading(new Vector2d(0,-31), Math.PI / 2)
      .build();

    barToFirstAllySample = drive.actionBuilder(new Pose2d(0, -31, Math.PI / 2))
      .setTangent(-Math.PI / 2)
      .splineToSplineHeading(new Pose2d(35, -41, Math.PI * (1.6 / 9.0)), 0)
      .build();

    firstAllySampleToObservationZone = drive.actionBuilder(new Pose2d(35, -41, Math.PI * (1.6 / 9.0)))
      .turnTo(-Math.PI * (3.0 / 9.0))
      .build();
/*
    observationZoneToSecondAllySample = drive.actionBuilder(new Pose2d(35, -39, -Math.PI * (2.0 / 9.0)))
      .strafeToSplineHeading(new Vector2d(45, -40),Math.PI * (2.0 / 9.0))
      .build();

    secondAllySampleToObservationZone = drive.actionBuilder(new Pose2d(45, -40, Math.PI * (2.0 / 9.0)))
      .turnTo(-Math.PI * (2.0 / 9.0))
      .build();

    observationZoneToThirdAllySample = drive.actionBuilder(new Pose2d(45, -40, -Math.PI * (2.0 / 9.0)))
      .strafeToSplineHeading(new Vector2d(55, -40),Math.PI * (2.0 / 9.0))
      .build();

    thirdAllySampleToObservationZone = drive.actionBuilder(new Pose2d(55, -40, Math.PI * (2.0 / 9.0)))
      .turnTo(-Math.PI * (2.0 / 9.0))
      .build();
      */

    observationZoneToSpecimenWallPos = drive.actionBuilder(new Pose2d(35, -41, -Math.PI * (3.0 / 9.0)))
        .strafeToSplineHeading(new Vector2d(48, -50), -Math.PI / 2)
        .strafeTo(new Vector2d(48, -60))
        .build();

    specimenWallPosToBar = drive.actionBuilder(new Pose2d(48, -60, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(4, -31), Math.PI / 2)
      .build();
/*
    barToSpecimenWallPos = drive.actionBuilder(new Pose2d(0, -31, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(40.5, -50), -Math.PI / 2)
      .build();

    specimenWallPosToBar2 = drive.actionBuilder(new Pose2d(40.5, -50, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(0, -31), Math.PI / 2)
      .build();

    barToSpecimenWallPos2 = drive.actionBuilder(new Pose2d(0, -31, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(40.5, -50), -Math.PI / 2)
      .build();

    specimenWallPosToBar3 = drive.actionBuilder(new Pose2d(40.5, -50, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(0, -31), Math.PI / 2)
      .build();

 */

    barToParkCorner = drive.actionBuilder(new Pose2d(4, -31, Math.PI / 2))
        .strafeTo(new Vector2d(62, -66))
        .build();

    rightSideTrajectories = new Action[] {rightStartToBar, barToFirstAllySample, firstAllySampleToObservationZone, observationZoneToSecondAllySample, secondAllySampleToObservationZone, observationZoneToThirdAllySample, thirdAllySampleToObservationZone, observationZoneToSpecimenWallPos, specimenWallPosToBar, barToSpecimenWallPos, specimenWallPosToBar2, barToSpecimenWallPos2, specimenWallPosToBar3, barToParkCorner};


    // TODO LEFT SIDE TRAJECTORIES ************
    drive = new MecanumDrive(myHardwareMap, new Pose2d(-39, -65, Math.PI / 2));

    leftStartToNet = drive.actionBuilder(new Pose2d(-39, -65, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-61, -48), Math.PI / 2) // was -62
      .build(); // Y was -49.5

    netToFirstYellowSample = drive.actionBuilder(new Pose2d(-61, -48, Math.PI /2))
      .strafeToSplineHeading(new Vector2d(-46, -49.5), Math.PI / 2)
      .build(); // TODO was -46

    firstYellowSampleToNet = drive.actionBuilder(new Pose2d(-46, -49.5, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-61, -48), Math.PI / 2)
      .build(); // TODO was -62 -49.5

    netToSecondYellowSample = drive.actionBuilder(new Pose2d(-61, -48, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-57.5, -49.5), Math.PI / 2)
      .build();

    secondYellowSampleToNet = drive.actionBuilder(new Pose2d(-56.5, -49.5, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-61, -48), Math.PI / 2)
      .build(); // TODO was -62 -49.5


    netToThirdYellowSampleWall = drive.actionBuilder(new Pose2d(-61, -48, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-52, -48), Math.PI-(Math.PI/4))
      .build();

    thirdYellowSampleWallToNet = drive.actionBuilder(new Pose2d(-52, -48, Math.PI-(Math.PI/4)))
      .strafeToSplineHeading(new Vector2d(-62, -49.5), Math.PI / 2)
      .build();

    leftSideTrajectories = new Action[] {leftStartToNet, netToFirstYellowSample, firstYellowSampleToNet, netToSecondYellowSample, secondYellowSampleToNet, netToThirdYellowSampleWall, thirdYellowSampleWallToNet};
  }

  public Action[] getRightSideTrajectories() {
    return rightSideTrajectories;
  }
  public Action[] getLeftSideTrajectories() {
    return leftSideTrajectories;
  }
}
