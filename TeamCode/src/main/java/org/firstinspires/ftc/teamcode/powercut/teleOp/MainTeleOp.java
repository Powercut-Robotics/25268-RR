package org.firstinspires.ftc.teamcode.powercut.teleOp;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;
import org.firstinspires.ftc.teamcode.powercut.vision.VisionSystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="Drive")
public class MainTeleOp extends OpMode {
    // Declaring the system
    private RobotSettings settings = new RobotSettings();
    private Drivetrain drivetrain = new Drivetrain();
    private ArmSystem arm = new ArmSystem();
    public DroneSystem droneSystem = new DroneSystem();
    public VisionSystem visionSystem = new VisionSystem();


    // Game monitoring
    public boolean isEndGame = false;
    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime loopTime = new ElapsedTime();

    // Actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();


    //sys mon
    private double armPos = 0.0;
    private double wristPos = 0.0;


    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
        droneSystem.init(hardwareMap);
        visionSystem.init(hardwareMap);
        droneSystem.preset();
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        drivetrain.doPowerFromGamepad(axial, lateral, yaw, getSpeedModifier());
        doArmControl();
        droneControl();

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;



        // Telemetry
        dash.sendTelemetryPacket(packet);
        updateTelemetry();
        loopTime.reset();

    }

    public void doArmControl() {
        double armSpeed = -gamepad2.left_stick_y;
        double wristSpeed = gamepad2.right_stick_y;

        if (Math.abs(armSpeed) > RobotSettings.manualArmControlDeadband || Math.abs(wristSpeed) > RobotSettings.manualWristControlDeadband) {
            runningActions.clear();
            arm.setArmPower(armSpeed);
            arm.setWristPower(wristSpeed * 0.5);
        } else if (gamepad2.triangle || gamepad2.circle || gamepad2.cross || gamepad2.square) {
            presetArmControl();
        } else {
            arm.stop();
        }

        if (gamepad2.dpad_down) {
            arm.gripLeftActivate();
            arm.gripRightActivate();
        } else if (gamepad2.dpad_up) {
            arm.gripLeftRelease();
            arm.gripRightRelease();
        } else if (gamepad2.dpad_right) {
            arm.gripLeftTuck();
            arm.gripRightTuck();
        }
    }

    private void presetArmControl() {
        if (gamepad2.triangle) {
            runningActions.clear();
            runningActions.add(arm.armUp());
        } else if (gamepad2.circle) {
            runningActions.clear();
            runningActions.add(
                    new ParallelAction(
                            arm.armDown(),
                            arm.gripTuck()
                    )

            );
        } else if (gamepad2.cross) {
            runningActions.clear();
            runningActions.add(
                    new ParallelAction(
                            arm.armDown(),
                            arm.gripRelease()
                    )
            );
        } else if (gamepad2.square) {
            runningActions.clear();
        }
    }

    private void droneControl() {
        if (gamepad1.triangle && isEndGame) {
            droneSystem.doLaunch();
        }
    }

    private double getSpeedModifier(){
        double speedMultiplier = RobotSettings.totalSpeedModifier;
        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            speedMultiplier = RobotSettings.slowmodeSpeedModifier;
        }

        return speedMultiplier;
    }
    private void updateTelemetry(){
        endgameCheck();

        List<AprilTagDetection> detections = visionSystem.getAprilTags();

        for (AprilTagDetection detection : detections) {
            try {
                telemetry.addData("===== Detected ID", detection.id);
                telemetry.addData("Pose", "%4.2f, %4.2f, %5.2f, %5.2f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw, detection.ftcPose.bearing);
                telemetry.addData("Distance", detection.ftcPose.range);
                telemetry.addData("Name", detection.metadata.name);
                telemetry.addData("Field Pos", "%4.2f, %4.2f", detection.metadata.fieldPosition.get(0), detection.metadata.fieldPosition.get(1));
            } catch (Exception e) {
                telemetry.addData("Error:", e.getMessage());
            }

        }

        double[] powers = drivetrain.getPowers();
        armPos = arm.armMotor.getCurrentPosition();
        wristPos = arm.wristMotor.getCurrentPosition();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm Position", armPos);
        packet.put("Wrist Position", wristPos);

        dash.sendTelemetryPacket(packet);

        telemetry.addData("Wheel Powers:", "%2.1f, %2.1f, %2.1f, %2.1f", powers[0], powers[1], powers[2], powers[3]);
        telemetry.addData("Loop time:", "%4.2f", loopTime.milliseconds());
        telemetry.addData("Arm/Wrist Position:", "%5.1f, %5.1f", armPos, wristPos);

        telemetry.update();
    }

    private void endgameCheck(){
        if (runtime.seconds() > 90 && runtime.seconds() < 90.1){
            isEndGame = true;

            gamepad1.rumble(settings.endgameRumbleTime);
            gamepad2.rumble(settings.endgameRumbleTime);

            gamepad1.setLedColor(255.0, 0.0,0.0, 30000);
            gamepad2.setLedColor(255.0, 0.0, 0.0, 30000);
        }
    }


}
