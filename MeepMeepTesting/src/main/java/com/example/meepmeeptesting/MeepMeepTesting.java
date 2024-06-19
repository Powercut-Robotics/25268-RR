package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity frontBlueCentre = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 63.5, Math.toRadians(270)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-57, 40), Math.toRadians(180))
                                .strafeTo(new Vector2d(-57,6))
                                .strafeTo(new Vector2d(50, 6))
                                .strafeTo(new Vector2d(52, 36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(52, 5))
                                .strafeTo(new Vector2d(62, 5))
                                .build()
                );

        RoadRunnerBotEntity frontBlueSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 63.5, Math.toRadians(270)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-57, 40), Math.toRadians(180))
                                .strafeTo(new Vector2d(-57,6))
                                .strafeTo(new Vector2d(50, 6))
                                .strafeTo(new Vector2d(52, 36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(52, 63))
                                .strafeTo(new Vector2d(62, 63))
                                .build()
                );

        RoadRunnerBotEntity frontBlueRiggingSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 63.5, Math.toRadians(270)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-24, 58), Math.toRadians(0))
                                .splineTo(new Vector2d(12, 58), Math.toRadians(0))
                                .splineTo(new Vector2d(40, 36), Math.toRadians(180))
                                .strafeTo(new Vector2d(52, 36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(52, 63))
                                .strafeTo(new Vector2d(62, 63))
                                .build()
                );

        RoadRunnerBotEntity backBlueSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 63.5, Math.toRadians(270)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(40, 36), Math.toRadians(180))
                                .strafeTo(new Vector2d(52,36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(52, 63))
                                .strafeTo(new Vector2d(62, 63))
                                .build()
                );

        RoadRunnerBotEntity backBlueCentre = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 63.5, Math.toRadians(270)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(40, 36), Math.toRadians(180))
                                .strafeTo(new Vector2d(52,36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(52, 5))
                                .strafeTo(new Vector2d(62, 5))
                                .build()
                );

        RoadRunnerBotEntity backRedSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.5, Math.toRadians(90)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(40, -36), Math.toRadians(180))
                                .strafeTo(new Vector2d(52,-36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(52, -63))
                                .strafeTo(new Vector2d(62, -63))
                                .build()
                );

        RoadRunnerBotEntity backRedCentre = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -63.5, Math.toRadians(90)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(40, -36), Math.toRadians(180))
                                .strafeTo(new Vector2d(52,-36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(52, -5))
                                .strafeTo(new Vector2d(62, -5))
                                .build()
                );

        RoadRunnerBotEntity frontRedCentre = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63.5, Math.toRadians(90)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-57, -40), Math.toRadians(180))
                                .strafeTo(new Vector2d(-57,-6))
                                .strafeTo(new Vector2d(50, -6))
                                .strafeTo(new Vector2d(52, -36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(50, -5))
                                .strafeTo(new Vector2d(62, -5))
                                .build()
                );

        RoadRunnerBotEntity frontRedSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63.5, Math.toRadians(90)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-57, -40), Math.toRadians(180))
                                .strafeTo(new Vector2d(-57,-6))
                                .strafeTo(new Vector2d(50, -6))
                                .strafeTo(new Vector2d(52, -36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(50, -63))
                                .strafeTo(new Vector2d(62, -63))
                                .build()
                );

        RoadRunnerBotEntity frontRedRiggingSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63.5, Math.toRadians(90)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-24, -58), Math.toRadians(0))
                                .splineTo(new Vector2d(12, -58), Math.toRadians(0))
                                .splineTo(new Vector2d(40, -36), Math.toRadians(180))
                                .strafeTo(new Vector2d(52, -36))
                                .waitSeconds(3.5)
                                .strafeTo(new Vector2d(50, -63))
                                .strafeTo(new Vector2d(62, -63))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(frontBlueCentre)
                .addEntity(frontBlueSide)
                .addEntity(frontBlueRiggingSide)
                .addEntity(backBlueSide)
                .addEntity(backBlueCentre)
                .addEntity(frontRedCentre)
                .addEntity(frontRedRiggingSide)
                .addEntity(frontRedSide)
                .addEntity(backRedSide)
                .addEntity(backRedCentre)

                .start();
    }
}