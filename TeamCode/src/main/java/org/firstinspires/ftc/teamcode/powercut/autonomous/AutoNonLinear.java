package org.firstinspires.ftc.teamcode.powercut.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;
import org.firstinspires.ftc.teamcode.powercut.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Disabled
@Config
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

        telemetry.addLine("Init hardware maps");
        telemetry.update();

        arm.gripLeftActivate();
        arm.gripRightActivate();
        //droneSystem.preset();

        telemetry.addLine("Init hardware positions");
        telemetry.update();



        lookLeftSM = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(0, -40), Math.toRadians(120))
                .waitSeconds(0.5)
                .build();

        turnToCentreSM = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(8, -24), Math.toRadians(0))
                .waitSeconds(0.5)
                .build();

        turnToRightSM = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(14, -30), Math.toRadians(-30))
                .waitSeconds(0.5)
                .build();

        telemetry.addLine("Init paths. Fully Initialised.");
        telemetry.update();

    }

    @Override
    public void start() {
        telemetry.addLine("Pathing to left SM in progress.");
        telemetry.update();

        Actions.runBlocking(lookLeftSM);

        telemetry.addLine("Path to left SM complete.");
        telemetry.update();

        Actions.runBlocking(new SleepAction(1000));
        if (visionSystem.isGamepeicePresent()) {
            telemetry.addLine("Gamepeice at SM Left. Placing Pixel");
            telemetry.update();
            position = 1;
            Actions.runBlocking(
                    new SequentialAction(
                            arm.armDown(),
                            arm.gripLeftReleaseAction()
                    )
            );
        } else {
            telemetry.addLine("Pathing to centre SM in progress.");
            telemetry.update();
            Actions.runBlocking(turnToCentreSM);

            telemetry.addLine("Path to centre SM Complete.");
            telemetry.update();

            if (visionSystem.isGamepeicePresent()) {
                position = 2;
                telemetry.addLine("Gamepeice at SM Centre. Placing Pixel");
                telemetry.update();
                Actions.runBlocking(
                        new SequentialAction(
                                arm.armDown(),
                                arm.gripLeftReleaseAction()
                        )
                );
            } else {
                position = 3;
                telemetry.addLine("Gamepeice at SM Right (Proc Of Elim). Placing Pixel");
                telemetry.update();
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
