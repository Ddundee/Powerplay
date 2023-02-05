package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RightPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPos = new Pose2d(38, 58, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(startPos)
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15.5)
                .setConstraints(30, 30, 30, 30, 15.5)
                .setDimensions(16, 17)
                .setDriveTrainType(DriveTrainType.MECANUM)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(32.75, -61.25, Math.toRadians(90)))
                                        .lineTo(new Vector2d(36, -12))
                                        .turn(Math.toRadians(45))

                                        //Where we drop preload cone
                                        .lineTo(new Vector2d(31, -7))
                                        .waitSeconds(2)
                                        .lineTo(new Vector2d(36, -12))
                                        .turn(Math.toRadians(45))

//                                      This is where we pick up cones
                                        .lineTo(new Vector2d(60, -12))
                                        .waitSeconds(1)
                                        .lineTo(new Vector2d(36, -12))
                                        .turn(Math.toRadians(-45))
                                        .lineTo(new Vector2d(31, -7))
                                        .waitSeconds(2)

//                                        A cycle
                                        .lineTo(new Vector2d(36, -12))
                                        .turn(Math.toRadians(45))
                                        .lineTo(new Vector2d(60, -12))
                                        .waitSeconds(1)
                                        .lineTo(new Vector2d(36, -12))
                                        .turn(Math.toRadians(-45))
                                        .lineTo(new Vector2d(31, -7))
                                        .waitSeconds(2)
                                        .lineTo(new Vector2d(36, -12))

//                                        Go to parking position: LEFT
//                                        .turn(Math.toRadians(45))
//                                        .splineTo(new Vector2d(24, -12), Math.toRadians(180))
//                                        .splineTo(new Vector2d(12, -24), Math.toRadians(270))
//                                        .splineTo(new Vector2d(12, -36), Math.toRadians(270))

//                                        Go to parking position: MIDDLE
//                                        .turn(Math.toRadians(45))
//                                        Nothing

//                                         Go to parking position: RIGHT
                                        .turn(Math.toRadians(-135))
                                        .splineTo(new Vector2d(48, -12), Math.toRadians(0))
                                        .splineTo(new Vector2d(60, -24), Math.toRadians(270))
                                        .splineTo(new Vector2d(60, -36), Math.toRadians(270))

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}