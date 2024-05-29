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

        int position = 0;

        visionSystem.init(hardwareMap);

        arm.gripLeftActivate();
        arm.gripRightActivate();

        Action lookLeftSM = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(11.33, -45.32), Math.toRadians(117.55))
                .build();

        Action lookCentrelSM = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(8.26, -34.31), Math.toRadians(63.43))
                .build();


        Action trajectoryActionLeft;
        Action trajectoryActionCentre;
        Action trajectoryActionRight;

        trajectoryActionLeft = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(5, -35), Math.toRadians(120))
                .build();
        trajectoryActionCentre = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(12, -35), Math.toRadians(90))
                .build();
        trajectoryActionRight = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(19, -35), Math.toRadians(60))
                .build();

        Action trajToBackLeft = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(51, -30), Math.toRadians(180))
                .build();

        Action trajToBackCentre = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(51, -35), Math.toRadians(180))
                .build();

        Action trajToBackRight = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(51, -40), Math.toRadians(180))
                .build();

        Action closeOut = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(60, -10), Math.toRadians(180))
                .build();

        int visionOutputPosition = visionSystem.getGamepeicePosition();

        while (!isStopRequested() && !opModeIsActive()) {

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
        }
        ;

        // runtime

        Actions.runBlocking(lookLeftSM);

        if (visionSystem.isGamepeicePresent()) {
            position = 1;
            telemetry.addData("pos", position);
        }


        if (position != 1) {
            Actions.runBlocking(lookCentrelSM);

            if (visionSystem.isGamepeicePresent()) {
                position = 2;
                telemetry.addData("pos", position);
            } else {
                position = 3;
                telemetry.addData("pos", position);
            }


        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {

            telemetry.addData("Pose:", "%4.2f, %4.2f, %4.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.real);

            telemetry.update();
        }
    }
}

