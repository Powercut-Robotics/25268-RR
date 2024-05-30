package org.firstinspires.ftc.teamcode.powercut.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;
import org.firstinspires.ftc.teamcode.powercut.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "MainAuto", preselectTeleOp = "Drive")
public class AutoNonLinear extends OpMode {
    private MecanumDrive drive;
    private VisionSystem visionSystem = new VisionSystem();
    private ArmSystem arm = new ArmSystem();
    private DroneSystem droneSystem = new DroneSystem();

    // monitoring
    private int position = 0;

    private Action lookLeftSM;
    private Action turnToCentreSM;
    private Action turnToRightSM;
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(12, -63, Math.toRadians(90)));
        arm.init(hardwareMap);
        visionSystem.init(hardwareMap);
        droneSystem.init(hardwareMap);
        droneSystem.preset();

        Actions.runBlocking(arm.gripActivate());

        lookLeftSM = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(12, -36), Math.toRadians(90))
                .splineTo(new Vector2d(12, -36), Math.toRadians(0))
                .build();

        turnToCentreSM = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(12, -36), Math.toRadians(90))
                .build();

        turnToRightSM = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(12, -36), Math.toRadians(180))
                .build();


    }

    @Override
    public void start() {
        Actions.runBlocking(lookLeftSM);

        if (visionSystem.isGamepeicePresent()) {
            telemetry.addLine("Gamepeice at SM Left. Placing Pixel");
            position = 1;
            Actions.runBlocking(
                    new SequentialAction(
                            arm.armDown(),
                            arm.gripLeftReleaseAction()
                    )
            );
        } else {
            Actions.runBlocking(turnToCentreSM);
            if (visionSystem.isGamepeicePresent()) {
                position = 2;
                telemetry.addLine("Gamepeice at SM Centre. Placing Pixel");
                Actions.runBlocking(
                        new SequentialAction(
                                arm.armDown(),
                                arm.gripLeftReleaseAction()
                        )
                );
            } else {
                position = 3;
                telemetry.addLine("Gamepeice at SM Right (Proc Of Elim). Placing Pixel");
                Actions.runBlocking(
                        new SequentialAction(
                                turnToRightSM,
                                arm.armDown(),
                                arm.gripLeftReleaseAction()
                        )
                );
            }
        }

        // to backdrop
    }

    @Override
    public void loop() {
        telemetry.addData("Pose", "%4.2f, %4.2f, %4.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.real);
        telemetry.addData("Detected position", position);
        telemetry.update();
    }
}
