package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;

public class RRPushTrajectories {

  HardwareMap myHardwareMap;

  MecanumDrive drive;

  // Observation Side

  public Action rightStartToBar;
  public Action barToObservationZoneAnd3Samples;
  public Action observationZoneToSpecimenWallPos;
  public Action specimenWallPosToBar;
  public Action barToSpecimenWallPos;
  public Action specimenWallPosToBar2;
  public Action barToSpecimenWallPos2;
  public Action specimenWallPosToBar3;
  public Action barToSpecimenWallPos3;
  public Action specimenWallPosToBar4;
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
  public Action netToAscentZone;

  public Action[] leftSideTrajectories;

  public RRPushTrajectories(HardwareMap hardwareMap) {
    myHardwareMap = hardwareMap;
  }
  public void initTrajectories() {

    drive = new MecanumDrive(myHardwareMap, new Pose2d(24, -65 ,Math.PI / 2));

    rightStartToBar = drive.actionBuilder(new Pose2d(24, -64, Math.PI / 2))
      .splineToConstantHeading(new Vector2d(0,-36), Math.PI / 2) // 0,-31 -> -36
      .build();

    barToObservationZoneAnd3Samples = drive.actionBuilder(new Pose2d(0, -36, Math.PI / 2)) // 0,-31 -> 0,-36
      .setTangent(-Math.PI / 2)
      .splineToConstantHeading(new Vector2d(40, -36), Math.PI / 2, new TranslationalVelConstraint(30)) //was -24
      .splineToConstantHeading(new Vector2d(45, -18), 0, new TranslationalVelConstraint(28))
      .splineToConstantHeading(new Vector2d(51.5, -24), -Math.PI / 2, new TranslationalVelConstraint(28))
      .splineToConstantHeading(new Vector2d(51.5, -53), -Math.PI / 2, new TranslationalVelConstraint(33)) // TODO
      .splineToConstantHeading(new Vector2d(51.5, -24), Math.PI / 2, new TranslationalVelConstraint(28))
      .splineToConstantHeading(new Vector2d(56.5, -18), 0, new TranslationalVelConstraint(28))
      .splineToConstantHeading(new Vector2d(62, -24), -Math.PI / 2, new TranslationalVelConstraint(28))
      .splineToConstantHeading(new Vector2d(62, -54), -Math.PI / 2, new TranslationalVelConstraint(33)) // TODO
      //.splineToConstantHeading(new Vector2d(60, -24), Math.PI / 2, new TranslationalVelConstraint(20))
      //.splineToConstantHeading(new Vector2d(62, -15), 0, new TranslationalVelConstraint(20))
      //.splineToConstantHeading(new Vector2d(67, -24), -Math.PI / 2, new TranslationalVelConstraint(20))
      //.splineToConstantHeading(new Vector2d(67, -51), -Math.PI / 2, new TranslationalVelConstraint(20))
      .splineToSplineHeading(new Pose2d(48, -50, -Math.PI / 2), -Math.PI / 2, new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(20), new AngularVelConstraint(Math.PI * 2 / 3)))) // TODO
      // -63 -> -62
      .strafeTo(new Vector2d(48, -62.25), new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(12), new AngularVelConstraint(Math.PI * 2 / 3)))) // TODO 1/3->2/3 do we need ang constraint here?
      .build();

    // SECOND SPECIMEN
    specimenWallPosToBar = drive.actionBuilder(new Pose2d(48, -62.25, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(2, -38.5), Math.PI / 2) // was -38 was 38.2
      .build();

    // SECOND SPEC DONE BACK TO WALL
    barToSpecimenWallPos = drive.actionBuilder(new Pose2d(2, -38.5, Math.PI / 2)) // TODO was 38.2
      .setTangent(-Math.PI / 2)
      .splineToSplineHeading(new Pose2d(48, -50, -Math.PI / 2), 0)
      // 66 -> 64
      .strafeTo(new Vector2d(48, -64.25))
      .build();

    // THIRD SPECIMEN
    specimenWallPosToBar2 = drive.actionBuilder(new Pose2d(48, -64.25, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(4, -41.25), Math.PI / 2) // TODO -38 -> -40
      .build();

    /*
    // THIRD SPECIMEN BACK TO WALL
    barToSpecimenWallPos2 = drive.actionBuilder(new Pose2d(4, -42.5, Math.PI / 2)) // TODO -38.2 -> -40
      .strafeToSplineHeading(new Vector2d(48, -50), -Math.PI / 2)
      .strafeTo(new Vector2d(48, -64))
      .build();

    // 4TH SPECIMEN
    specimenWallPosToBar3 = drive.actionBuilder(new Pose2d(48, -64, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(6, -38.2), Math.PI / 2) // was 0:-30
      .build();

    // 4TH DONE BACK TO WALL
    barToSpecimenWallPos3 = drive.actionBuilder(new Pose2d(6, -36, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(48, -64), -Math.PI / 2) // 40.5,-50
      .strafeTo(new Vector2d(48, -64))
      .build();

    specimenWallPosToBar4 = drive.actionBuilder(new Pose2d(40.5, -50, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(0, -31), Math.PI / 2)
      .build();
*/
    // 4,-31 -> 6,-36
    barToParkCorner = drive.actionBuilder(new Pose2d(4, -41.25, Math.PI / 2))
        .strafeTo(new Vector2d(60, -72))
        .build();

    rightSideTrajectories = new Action[] {rightStartToBar, barToObservationZoneAnd3Samples, specimenWallPosToBar, barToSpecimenWallPos, specimenWallPosToBar2, barToSpecimenWallPos2, specimenWallPosToBar3, barToSpecimenWallPos3, specimenWallPosToBar4, barToParkCorner};

    drive = new MecanumDrive(myHardwareMap, new Pose2d(-39, -65, Math.PI / 2));

    leftStartToNet = drive.actionBuilder(new Pose2d(-39, -65, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-54, -54), -Math.PI * (3.0 / 4.0))
      .build();

    netToFirstYellowSample = drive.actionBuilder(new Pose2d(-54, -54, -Math.PI * (3.0/4.0)))
        .strafeToSplineHeading(new Vector2d(-48, -35), Math.PI / 2)
        .build();

    firstYellowSampleToNet = drive.actionBuilder(new Pose2d(-48, -35, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-54, -54), -Math.PI * (3.0 / 4.0))
      .build();

    netToSecondYellowSample = drive.actionBuilder(new Pose2d(-54, -54, -Math.PI * (3.0 / 4.0)))
      .strafeToSplineHeading(new Vector2d(-60, -35), Math.PI / 2)
      .build();

    secondYellowSampleToNet = drive.actionBuilder(new Pose2d(-60, -35, Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(-54, -54), -Math.PI * (3.0 / 4.0))
      .build();

    netToThirdYellowSampleWall = drive.actionBuilder(new Pose2d(-54, -54, -Math.PI * (3.0 / 4.0)))
      .strafeToSplineHeading(new Vector2d(-60, -26), Math.PI)
      .build();

    thirdYellowSampleWallToNet = drive.actionBuilder(new Pose2d(-60, -26, Math.PI))
      .strafeToSplineHeading(new Vector2d(-54, -54), -Math.PI * (3.0 / 4.0))
      .build();

    netToAscentZone = drive.actionBuilder(new Pose2d(-54, -54, -Math.PI * (3.0 / 4.0)))
      .setTangent(Math.PI / 2)
      .splineToSplineHeading(new Pose2d(-54, -12, 0), Math.PI / 2)
      .splineToConstantHeading(new Vector2d(-24, 0), 0)
      .build();

    leftSideTrajectories = new Action[] {leftStartToNet, netToFirstYellowSample, firstYellowSampleToNet, netToSecondYellowSample, secondYellowSampleToNet, netToThirdYellowSampleWall, thirdYellowSampleWallToNet, netToAscentZone};
  }

  public Action[] getRightSideTrajectories() {
    return rightSideTrajectories;
  }
  public Action[] getLeftSideTrajectories() {
    return leftSideTrajectories;
  }
}
