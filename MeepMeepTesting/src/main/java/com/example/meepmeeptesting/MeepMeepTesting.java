package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity frontBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 64, Math.toRadians(270)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-58, 40), Math.toRadians(180))
                                .strafeTo(new Vector2d(-58,10))
                                .strafeTo(new Vector2d(50, 10))
                                .strafeTo(new Vector2d(52, 36))
                                .waitSeconds(4)
                .strafeTo(new Vector2d(52, 10))
                .strafeTo(new Vector2d(62, 10))
                                .build()
                );

        RoadRunnerBotEntity backBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 64, Math.toRadians(270)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(40, 36), Math.toRadians(180))
                                .strafeTo(new Vector2d(52,36))
                                .waitSeconds(4)
                                .strafeTo(new Vector2d(52, 10))
                                .strafeTo(new Vector2d(62, 10))
                                .build()
                );

        RoadRunnerBotEntity backRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -64, Math.toRadians(90)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(40, -36), Math.toRadians(180))
                                .strafeTo(new Vector2d(52,-36))
                                .waitSeconds(4)
                                .strafeTo(new Vector2d(52, -60))
                                .strafeTo(new Vector2d(62, -60))
                                .build()
                );

        RoadRunnerBotEntity frontRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -64, Math.toRadians(90)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-58, -40), Math.toRadians(180))
                                .strafeTo(new Vector2d(-58,-10))
                                .strafeTo(new Vector2d(50, -10))
                                .strafeTo(new Vector2d(52, -36))
                                .waitSeconds(4)
                                .strafeTo(new Vector2d(50, -10))
                                .strafeTo(new Vector2d(62, -10))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(frontBlue)
                .addEntity(backBlue)
                .addEntity(frontRed)
                .addEntity(backRed)
                .start();
    }
}