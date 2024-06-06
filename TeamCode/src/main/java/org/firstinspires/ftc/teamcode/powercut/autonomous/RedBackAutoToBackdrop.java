package org.firstinspires.ftc.teamcode.powercut.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.powercut.hardware.ArmActions;
import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;
import org.firstinspires.ftc.teamcode.powercut.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.powercut.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous(name = "BackRedAutoToBackdrop", preselectTeleOp = "Drive")
public class RedBackAutoToBackdrop extends OpMode {
    private MecanumDrive drive;
    private VisionSystem visionSystem = new VisionSystem();
    private ArmSystem arm = new ArmSystem();
    private ArmActions armActions = new ArmActions();
    private DroneSystem droneSystem = new DroneSystem();

    //Paths

    private Action toLeftSpike;
    private Action toCentreSpike;
    private Action toRightSpike;

    private Action toLeftBackdrop;
    private Action toCentreBackdrop;
    private Action toRightBackdrop;

    private Action parkFromLeft;
    private Action parkFromCentre;
    private Action parkFromRight;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(12, -63, Math.toRadians(90)));
        arm.init(hardwareMap);
        visionSystem.init(hardwareMap);
        droneSystem.init(hardwareMap);

        telemetry.addLine("Init hardware maps");
        telemetry.update();

        arm.doPresetArm();
        arm.doPresetWrist();
        arm.gripLeftActivate();
        arm.gripRightActivate();
        droneSystem.preset();

        telemetry.addLine("Init hardware positions");
        telemetry.update();

        toLeftSpike = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(7, -38), Math.toRadians(135.00))
                .build();

        toCentreSpike = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(12, -32), Math.toRadians(90))
                .build();

        toRightSpike = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(17, -38), Math.toRadians(45))
                .build();

        toLeftBackdrop = drive.actionBuilder(new Pose2d(7, -38, Math.toRadians(135)))
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(51, -29))
                .build();

        toCentreBackdrop = drive.actionBuilder(new Pose2d(12, -32, Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .strafeTo(new Vector2d(51, -35))
                .build();

        toRightBackdrop = drive.actionBuilder(new Pose2d(17, -38, Math.toRadians(45)))
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d(51, -42))
                .build();

        parkFromLeft = drive.actionBuilder(new Pose2d(51, -29, Math.toRadians(180)))
                .strafeTo(new Vector2d(50, -12))
                .strafeTo(new Vector2d(60, -12))
                .build();

        parkFromCentre = drive.actionBuilder(new Pose2d(51, -35, Math.toRadians(180)))
                .strafeTo(new Vector2d(50, -12))
                .strafeTo(new Vector2d(60, -12))
                .build();

        parkFromRight = drive.actionBuilder(new Pose2d(51, -42, Math.toRadians(180)))
                .strafeTo(new Vector2d(50, -12))
                .strafeTo(new Vector2d(60, -12))
                .build();



        telemetry.addLine("Init paths. Fully Initialised.");
        telemetry.update();

    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", visionSystem.colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionSystem.visionSystem.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + visionSystem.colourMassDetectionProcessor.getLargestContourX() + ", y: " + visionSystem.colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", visionSystem.colourMassDetectionProcessor.getLargestContourArea());
        telemetry.update();
    }
    @Override
    public void start() {

        if (visionSystem.visionSystem.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionSystem.visionSystem.stopLiveView();
            visionSystem.visionSystem.stopStreaming();
        }

        // gets the recorded prop position
        ColourMassDetectionProcessor.PropPositions recordedPropPosition = visionSystem.colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
        }

        // now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
        switch (recordedPropPosition) {
            case LEFT:
                Actions.runBlocking(new SequentialAction(
                        new SleepAction(0.5),
                        toLeftSpike,
                        armActions.armDown(),
                        armActions.gripLeftTuckAction(),
                        new SleepAction(0.5),
                        toLeftBackdrop,
                        new SleepAction(0.5),
                        new ParallelAction(
                                armActions.armUp(),
                                armActions.wristUp()
                        ),
                        new SleepAction(0.5),
                        armActions.gripTuck(),
                        new SleepAction(0.5),
                        new ParallelAction(armActions.presetArm(), armActions.presetWrist(), armActions.gripRelease(), parkFromLeft)
                ));
                break;
            case MIDDLE:
                Actions.runBlocking(new SequentialAction(
                        new SleepAction(0.5),
                        toCentreSpike,
                        armActions.armDown(),
                        armActions.gripLeftTuckAction(),
                        new SleepAction(0.5),
                        toCentreBackdrop,
                        new SleepAction(0.5),
                        new ParallelAction(
                                armActions.armUp(),
                                armActions.wristUp()
                        ),
                        new SleepAction(0.5),
                        armActions.gripTuck(),
                        new SleepAction(0.5),
                        new ParallelAction(armActions.presetArm(), armActions.presetWrist(), armActions.gripRelease(), parkFromCentre)
                ));
                break;
            case RIGHT:
                Actions.runBlocking(new SequentialAction(
                        new SleepAction(0.5),
                        toRightSpike,
                        armActions.armDown(),
                        armActions.gripLeftTuckAction(),
                        new SleepAction(0.5),
                        toRightBackdrop,
                        new SleepAction(0.5),
                        new ParallelAction(
                                armActions.armUp(),
                                armActions.wristUp()
                        ),
                        new SleepAction(0.5),
                        armActions.gripTuck(),
                        new SleepAction(0.5),
                        new ParallelAction(armActions.presetArm(), armActions.presetWrist(), armActions.gripRelease(), parkFromRight)
                ));
                break;
        }


        // to backdrop
    }

    @Override
    public void loop() {
        telemetry.addData("Pose", "%4.2f, %4.2f, %4.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.real);
        telemetry.update();
    }
}
