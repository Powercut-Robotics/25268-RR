package org.firstinspires.ftc.teamcode.powercut.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Autonomous")
public class MainAutonomous extends LinearOpMode {
    private MecanumDrive drive;
    private VisionSystem visionSystem = new VisionSystem();


    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(12.39, -63, Math.toRadians(90)));
        ArmSystem arm = new ArmSystem();
        arm.init(hardwareMap);

        visionSystem.init(hardwareMap);

        arm.gripLeftActivate();
        arm.gripRightActivate();

        Action trajectoryActionLeft;
        Action trajectoryActionCentre;
        Action trajectoryActionRight;
        Action trajectoryActionCloseOut;

        trajectoryActionLeft = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(7.73, -33.88), Math.toRadians(164.48))
                .build();
        trajectoryActionCentre = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(12, -35), Math.toRadians(90))
                .build();
        trajectoryActionRight = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(20.86, -38.01), Math.toRadians(46.35))
                .build();

        Action trajToBackLeft = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(1,1), Math.toRadians(180))
                .build();

        Action trajToBackCentre = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(1,1), Math.toRadians(180))
                .build();

        Action trajToBackRight = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(1,1), Math.toRadians(180))
                .build();

        Action closeOut = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(1,1), Math.toRadians(0))
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

        if (isStopRequested()) return;
        Action trajectoryActionChosen;
        Action backTrajChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = trajectoryActionLeft;
            backTrajChosen = trajToBackLeft;
        } else if (startPosition == 2) {
            trajectoryActionChosen = trajectoryActionCentre;
            backTrajChosen = trajToBackCentre;
        } else if (startPosition == 3) {
            trajectoryActionChosen = trajectoryActionRight;
            backTrajChosen = trajToBackRight;
        } else {
            trajectoryActionChosen = trajectoryActionCentre;
            backTrajChosen = trajToBackCentre;
        };

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                trajectoryActionChosen,
                                arm.armDown()
                        ),
                        new SleepAction(0.5),
                        arm.gripLeftReleaseAction(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                backTrajChosen,
                                arm.armUp()
                        ),
                        new SleepAction(0.5),
                        arm.gripRightReleaseAction(),
                        closeOut,
                        arm.armToResetPosition()
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

