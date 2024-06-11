package org.firstinspires.ftc.teamcode.powercut.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "FrontBlueAutoToBackdrop", preselectTeleOp = "Drive")
public class BlueFrontAutoToBackdrop extends OpMode {
    private MecanumDrive drive;
    private ArmSystem arm = new ArmSystem();
    private DroneSystem droneSystem = new DroneSystem();

    // monitoring

    private Action toBackdrop;
    private Action park;
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(-36, 63, Math.toRadians(270)));
        arm.init(hardwareMap);
        droneSystem.init(hardwareMap);

        telemetry.addLine("Init hardware maps");
        telemetry.update();

        droneSystem.preset();
        arm.doPresetArm();
        arm.doPresetWrist();

        arm.gripLeftActivate();
        arm.gripRightActivate();

        telemetry.addLine("Init hardware positions");
        telemetry.update();



        toBackdrop = drive.actionBuilder(drive.pose)
                .lineToY(12)
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(50, 12))
                .strafeTo(new Vector2d(52, 36))
                .build();

        park = drive.actionBuilder(new Pose2d(52, 36, Math.toRadians(180)))
                .strafeTo(new Vector2d(52, 6))
                .strafeTo(new Vector2d(62, 6))
                .build();


        telemetry.addLine("Init paths. Fully Initialised.");
        telemetry.update();

    }

    @Override
    public void start() {
      Actions.runBlocking(new SequentialAction(
              new SleepAction(1),
              toBackdrop,
              new SleepAction(1),
              new ParallelAction(
                      arm.armUp(),
                      arm.wristUp()
              ),
              new SleepAction(1),
              arm.gripTuck(),
              new SleepAction(1),
              new ParallelAction(arm.presetArm(), arm.presetWrist(), arm.gripRelease(), park)
      ));

        // to backdrop
    }

    @Override
    public void loop() {
        telemetry.addData("Pose", "%4.2f, %4.2f, %4.1f", drive.pose.position.x, drive.pose.position.y, drive.pose.heading.real);
        telemetry.update();
    }
}
