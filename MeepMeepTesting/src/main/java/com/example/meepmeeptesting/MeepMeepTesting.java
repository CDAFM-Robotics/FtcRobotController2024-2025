package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting{
  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(625);

    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
      .setDimensions(15, 14)
      // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
      .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
      .setStartPose(new Pose2d(0, 0, 0))
      .build();


    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
        .setTangent(Math.PI / 2)
        .splineToSplineHeading(new Pose2d(-54, -12, 0), Math.PI / 2)
        .splineToConstantHeading(new Vector2d(-24, 0), 0)
        .build());

    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61, -48, Math.PI / 2))
            .strafeToSplineHeading(new Vector2d(-52, -48), Math.PI-(Math.PI/4))
            .build());



    meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
      .setDarkMode(true)
      .setBackgroundAlpha(0.95f)
      .addEntity(myBot)
      .start();
  }
}