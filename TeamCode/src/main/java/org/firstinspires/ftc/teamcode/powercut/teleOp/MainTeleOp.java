package org.firstinspires.ftc.teamcode.powercut.teleOp;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;


@TeleOp(name="Drive")
public class MainTeleOp extends OpMode {
    // Declaring the system
    private RobotSettings settings = new RobotSettings();
    private Drivetrain drivetrain = new Drivetrain();
    private ArmSystem arm = new ArmSystem();
    public DroneSystem droneSystem = new DroneSystem();

    private PIDEx armPID = new PIDEx(settings.armCoefficients);
    private PIDEx gripPID = new PIDEx(settings.wristCoefficients);


    // Game monitoring
    public boolean isEndGame = false;
    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime loopTime = new ElapsedTime();
    // System Monitoring



    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
        droneSystem.init(hardwareMap);
        droneSystem.preset();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        drivetrain.doPowerFromGamepad(axial, lateral, yaw, getSpeedModifier());
        doArmControl();
        droneControl();

        // Telemetry
        updateTelemetry();
        loopTime.reset();

    }

    public void doArmControl() {
        double armSpeed = gamepad2.right_trigger - gamepad2.left_trigger;
        double wristSpeed = gamepad2.right_stick_y;

        if (Math.abs(armSpeed) > settings.manualArmControlDeadband || Math.abs(wristSpeed) > settings.manualWristControlDeadband) {
            arm.stop();

            arm.setArmPower(armSpeed);
            arm.setWristPower(wristSpeed);
        } else if (gamepad2.triangle || gamepad2.circle || gamepad2.cross || gamepad2.square) {
            presetArmControl();
        } else {
            arm.stop();
        }

        if (gamepad2.dpad_left) {
            arm.gripActivate();
        } else if (gamepad2.dpad_right) {
            arm.gripRelease();
        }
    }

    private void presetArmControl() {
        if (gamepad2.triangle) {
            armPID.calculate(settings.armUpPosition, arm.armMotor.getCurrentPosition());
            gripPID.calculate(settings.wristUpPosition, arm.wristMotor.getCurrentPosition());
        } else if (gamepad2.circle) {
            armPID.calculate(settings.armIntakePosition, arm.armMotor.getCurrentPosition());
            gripPID.calculate(settings.wristIntakePosition, arm.wristMotor.getCurrentPosition());
        } else if (gamepad2.cross) {
            armPID.calculate(settings.armDownPosition, arm.armMotor.getCurrentPosition());
            gripPID.calculate(settings.wristDownPosition, arm.wristMotor.getCurrentPosition());
        } else if (gamepad2.square) {
            //
        }
    }

    private void droneControl() {
        if (gamepad1.triangle && isEndGame) {
            droneSystem.doLaunch();
        }
    }

    private double getSpeedModifier(){
        double speedMultiplier = settings.totalSpeedModifier;
        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            speedMultiplier = settings.slowmodeSpeedModifier;
        }

        return speedMultiplier;
    }
    private void updateTelemetry(){
        endgameCheck();

        double[] powers = drivetrain.getPowers();

        telemetry.addData("Wheel Powers:", "%2.1f, %2.1f, %2.1f, %2.1f", powers[0], powers[1], powers[2], powers[3]);
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
