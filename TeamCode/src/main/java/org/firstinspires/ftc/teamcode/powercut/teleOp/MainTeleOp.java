package org.firstinspires.ftc.teamcode.powercut.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="Drive")
public class MainTeleOp extends OpMode {
    // Declaring the system
    private final RobotSettings settings = new RobotSettings();
    private final Drivetrain drivetrain = new Drivetrain();
    private final ArmSystem arm = new ArmSystem();
    private DroneSystem droneSystem = new DroneSystem();


    // Game monitoring
    private boolean isEndGame = true;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();

    // Actions
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();


    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
        droneSystem.init(hardwareMap);

        telemetry.addLine("Initalised");
        telemetry.update();

    }


    @Override
    public void start() {
        arm.gripLeftTuck();
        arm.gripRightTuck();

        arm.doPresetWrist();
        arm.doPresetArm();
        droneSystem.preset();

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
        double armSpeed = (gamepad2.left_bumper || gamepad2.right_bumper) ? gamepad2.right_stick_y : 0;
        double wristSpeed = (gamepad2.left_bumper || gamepad2.right_bumper) ? -gamepad2.left_stick_y : 0;

        if (Math.abs(armSpeed) > RobotSettings.manualArmControlDeadband || Math.abs(wristSpeed) > RobotSettings.manualWristControlDeadband) {
            runningActions.clear();
            arm.setArmPower(armSpeed);
            arm.setWristPower(wristSpeed * RobotSettings.wristSpeedModifier);
        } else if (gamepad2.triangle || gamepad2.circle || gamepad2.cross || gamepad2.square || gamepad2.dpad_up ) {
            presetArmControl();
        } else {
            arm.stop();
        }

        if (gamepad2.dpad_left) {
            arm.gripLeftActivate();
            arm.gripRightActivate();
        } else if (gamepad2.dpad_right) {
            arm.gripLeftTuck();
            arm.gripRightTuck();
        }

        if (gamepad2.share && gamepad2.left_bumper && gamepad2.right_bumper) {
            runningActions.clear();
            runningActions.add(
                    new ParallelAction(
                            arm.presetArm(),
                            arm.presetWrist()
                    )
            );
        }
    }

    private void presetArmControl() {
        if (gamepad2.triangle) {
            runningActions.clear();
            runningActions.add(
                    new ParallelAction(
                            arm.armUp(),
                            arm.wristUp()
                    )
            );
        } else if (gamepad2.cross) {
            runningActions.clear();
            runningActions.add(
                    new ParallelAction(
                            arm.presetArm(),
                            arm.wristDown(),
                            arm.gripTuck()
                    )
            );
        } else if (gamepad2.square) {
            runningActions.clear();
        } else if (gamepad2.dpad_up) {
            runningActions.add(
                    arm.gripActivate()
            );
        }


    }

    private void droneControl() {
        if (gamepad1.triangle && isEndGame) {
            droneSystem.doLaunch();
        } else if (gamepad1.cross) {
            droneSystem.preset();

        }
    }

    private double getSpeedModifier(){
        double speedMultiplier = RobotSettings.totalSpeedModifier;
        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.left_stick_button || gamepad1.right_stick_button) {
            speedMultiplier = RobotSettings.slowmodeSpeedModifier;
        }

        return speedMultiplier;
    }
    private void updateTelemetry(){
        endgameCheck();

        double[] powers = drivetrain.getPowers();
        //sys mon
        double armPos = arm.armMotor.getCurrentPosition();
        double wristPos = arm.wristMotor.getCurrentPosition();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Arm Position", armPos);
        packet.put("Wrist Position", wristPos);

        dash.sendTelemetryPacket(packet);

        telemetry.addData("Wheel Powers:", "%4.3f, %4.3f, %4.3f, %4.3f", powers[0], powers[1], powers[2], powers[3]);
        telemetry.addData("Arm/Wrist Position:", "%5.1f, %5.1f", armPos, wristPos);
        telemetry.addData("Grip Positions (Left/Right)", "%3.2f, %3.2f", arm.gripLeft.getPosition(), arm.gripRight.getPosition());
        telemetry.addData("Loop time:", "%4.2f", loopTime.milliseconds());
        
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
