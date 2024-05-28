package org.firstinspires.ftc.teamcode.powercut.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Autonomous")
public class MainAutonomous extends LinearOpMode {
    private RobotSettings settings = new RobotSettings();
    private MecanumDrive drive;
    private VisionSystem visionSystem = new VisionSystem();


    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(12.39, -63, Math.toRadians(90)));
        ArmSystem arm = new ArmSystem();
        arm.init(hardwareMap);

        visionSystem.init(hardwareMap);

        Action trajectoryActionLeft;
        Action trajectoryActionCentre;
        Action trajectoryActionRight;
        Action trajectoryActionCloseOut;

        trajectoryActionLeft = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(7.73, -33.88), Math.toRadians(164.48))
                .build();
        trajectoryActionCentre = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(13.87, -32.51), Math.toRadians(92.06))
                .build();
        trajectoryActionRight = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(20.86, -38.01), Math.toRadians(46.35))
                .build();
        trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(48, 12))
                .build();

        Action trajectoryActionBackRight = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(0.00, 0.00), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(25.73, 29.75, Math.toRadians(170)), Math.toRadians(170))
                .splineToConstantHeading(new Vector2d(49.55, 29.75), Math.toRadians(170.00))
                .build();


        int visionOutputPosition = visionSystem.getGamepeicePosition();

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        updateTelemetry();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        Action trajectoryActionBackChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = trajectoryActionLeft;
            trajectoryActionBackChosen = null;
        } else if (startPosition == 2) {
            trajectoryActionChosen = trajectoryActionCentre;
            trajectoryActionBackChosen = null;
        } else if (startPosition == 3) {
            trajectoryActionChosen = trajectoryActionRight;
            trajectoryActionBackChosen = trajectoryActionBackRight;
        } else {
            trajectoryActionChosen = trajectoryActionCentre;
            trajectoryActionBackChosen = null;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        arm.armDown(),
                        new SleepAction(0.5),
                        arm.gripLeftReleaseAction(),
                        new SleepAction(0.5)
                )
        );

    }

    public void updateTelemetry() {
        while (opModeIsActive()) {

            telemetry.addData("Pose:", "%4.2f, %4.2f, %4.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.real);

            telemetry.update();
        }
    }
}

