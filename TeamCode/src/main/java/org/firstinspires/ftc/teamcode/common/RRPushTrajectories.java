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

   public MecanumDrive drive;

  // Observation Side

  public Action rightStartToBar;
  public Action barToObservationZoneAnd3Samples;
  public Action specimenWallPosToBar;
  public Action barToSpecimenWallPos;
  public Action specimenWallPosToBar2;
  public Action barToParkCorner;


  public Action[] rightSideTrajectories;

  // Sample Side


  public Action[] leftSideTrajectories;

  public RRPushTrajectories(HardwareMap hardwareMap) {
    myHardwareMap = hardwareMap;
  }
  public void initTrajectories() {

    // START
    drive = new MecanumDrive(myHardwareMap, new Pose2d(12, -64.5 ,Math.PI / 2));

    // HELD SPECIMEN TO BAR
    rightStartToBar = drive.actionBuilder(new Pose2d(12, -64.5, Math.PI / 2))
      .splineToConstantHeading(new Vector2d(10,-38.5), Math.PI / 2) // 0,-31 -> -36 -> -35 (new motor)
      .build();

    // PUSH 2 Samples BACK to WALL
//    barToObservationZoneAnd3Samples = drive.actionBuilder(new Pose2d(10, -38.5, Math.PI / 2)) // 0,-31 -> 0,-36
//      .setTangent(-Math.PI / 2) // TODO: 1:22 24Jan 50->53 (slightly more left)
//            // TODO 28 speed constraint on trajectory
//      .splineToConstantHeading(new Vector2d(35, -34), Math.PI / 2, new TranslationalVelConstraint(20)) //x:40 was -24
//      .splineToConstantHeading(new Vector2d(40, -15), 0, new TranslationalVelConstraint(20))
//      .splineToConstantHeading(new Vector2d(48, -24), -Math.PI / 2, new TranslationalVelConstraint(20))
//      .splineToConstantHeading(new Vector2d(48, -53), -Math.PI / 2, new TranslationalVelConstraint(20)) // TODO
//      .splineToConstantHeading(new Vector2d(43, -24), Math.PI / 2, new TranslationalVelConstraint(20))
//      .splineToConstantHeading(new Vector2d(50, -15), 0, new TranslationalVelConstraint(20))
//      .splineToConstantHeading(new Vector2d(59, -24), -Math.PI / 2, new TranslationalVelConstraint(20))
//      .splineToConstantHeading(new Vector2d(59, -53), -Math.PI / 2, new TranslationalVelConstraint(20)) // TODO
//            // TODO ------------- position for wall approach
//      .splineToSplineHeading(new Pose2d(48, -48, -Math.PI / 2), -Math.PI / 2, new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(25), new AngularVelConstraint(Math.PI * 2 / 3)))) // TODO
//      // -63 -> -62 // was -50 moved forward to hopefully not hit sample on spin.
//            // TODO -61.5 to 61 April 3, 2025
//      .strafeTo(new Vector2d(48, -61), new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10), new AngularVelConstraint(Math.PI * 2 / 3)))) // TODO 1/3->2/3 do we need ang constraint here?
//      .build();

    barToObservationZoneAnd3Samples = drive.actionBuilder(new Pose2d(10, -38.5, Math.PI / 2)) // 0,-31 -> 0,-36
      .setTangent(-Math.PI / 2) // TODO: 1:22 24Jan 50->53 (slightly more left)
      // TODO 28 speed constraint on trajectory
      .splineToConstantHeading(new Vector2d(35, -34), Math.PI / 2, new TranslationalVelConstraint(20)) //x:40 was -24
      .splineToConstantHeading(new Vector2d(40, -15), 0, new TranslationalVelConstraint(20))
      .splineToConstantHeading(new Vector2d(48, -24), -Math.PI / 2, new TranslationalVelConstraint(20))
      .splineToConstantHeading(new Vector2d(48, -53), -Math.PI / 2, new TranslationalVelConstraint(20)) // TODO
      .splineToConstantHeading(new Vector2d(43, -24), Math.PI / 2, new TranslationalVelConstraint(20))
      .splineToConstantHeading(new Vector2d(50, -15), 0, new TranslationalVelConstraint(20))
      .splineToConstantHeading(new Vector2d(59, -24), -Math.PI / 2, new TranslationalVelConstraint(20))
      .splineToConstantHeading(new Vector2d(59, -53), -Math.PI / 2, new TranslationalVelConstraint(20)) // TODO
      // TODO ------------- position for wall approach
      .splineToSplineHeading(new Pose2d(48, -48, -Math.PI / 2), -Math.PI / 2 /*, new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(25), new AngularVelConstraint(Math.PI * 2 / 3)))*/ ) // TODO
      // -63 -> -62 // was -50 moved forward to hopefully not hit sample on spin.
      // TODO REMOVE FOR JUST IN TIME TRAJECTORY
      .strafeTo(new Vector2d(48, -55), new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(10), new AngularVelConstraint(Math.PI * 2 / 3)))) // TODO 1/3->2/3 do we need ang constraint here?
      .build();


    // SPEC2 TO BAR
    specimenWallPosToBar = drive.actionBuilder(new Pose2d(48, -61, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(7, -47.5), Math.PI / 2) // was -38 was 38.2
      .build();

    // BACK TO WALL
    barToSpecimenWallPos = drive.actionBuilder(new Pose2d(7, -40.5, Math.PI / 2)) // TODO was -38.2
      .setTangent(-Math.PI / 2)
      .splineToSplineHeading(new Pose2d(48, -50, -Math.PI / 2), 0)
      .strafeTo(new Vector2d(48, -61))
      .build();

    // SPEC3 TO BAR
    specimenWallPosToBar2 = drive.actionBuilder(new Pose2d(48, -59, -Math.PI / 2))
      .strafeToSplineHeading(new Vector2d(3, -53.5), Math.PI / 2) // TODO -38 -> -40
//      .strafeTo(new Vector2d(6, -43.5))  // TODO try to stop arm hitting bar
      .build();


    // *** PARK ***
    barToParkCorner = drive.actionBuilder(new Pose2d(6, -43.5, Math.PI / 2))
        .strafeTo(new Vector2d(62.5, -62))
        .build();


    //rightSideTrajectories = new Action[] {rightStartToBar, barToObservationZoneAnd3Samples, specimenWallPosToBar, barToSpecimenWallPos, specimenWallPosToBar2, barToSpecimenWallPos2, specimenWallPosToBar3, barToSpecimenWallPos3, specimenWallPosToBar4, barToParkCorner};

    /*
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

     */
  }

  public Action[] getRightSideTrajectories() {
    return rightSideTrajectories;
  }

}
