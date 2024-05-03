package org.firstinspires.ftc.teamcode.powercut.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.control.PIDController;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;
import org.firstinspires.ftc.teamcode.powercut.teleOp.logic.ArmControlLogic;
import org.firstinspires.ftc.teamcode.powercut.telemetry.MainTelemetry;


@TeleOp(name="Drive")
public class MainTeleOp extends OpMode {
    // Declaring the system
    private RobotSettings settings = new RobotSettings();
    private ArmControlLogic armController = new ArmControlLogic(hardwareMap);
    private Drivetrain drivetrain = settings.drivetrain;
    public DroneSystem droneSystem = new DroneSystem();



    // Declaring PID controllers
    private PIDController armPIDController = settings.armPIDController;
    private PIDController wristPIDController = settings.wristPIDController;

    // Game monitoring
    public boolean isEndGame = false;
    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime loopTime = new ElapsedTime();
    private MainTelemetry mainTelemetry = new MainTelemetry(gamepad1, gamepad2, loopTime, runtime, telemetry);
    // System Monitoring



    @Override
    public void init() {
        drivetrain.init(hardwareMap);
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
        armController.doArmControl();
        droneControl();

        // Telemetry
        MainTelemetry.updateTelemetry();
        loopTime.reset();

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
