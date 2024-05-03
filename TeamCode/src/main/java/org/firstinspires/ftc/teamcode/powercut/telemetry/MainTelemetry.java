package org.firstinspires.ftc.teamcode.powercut.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.teleOp.MainTeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MainTelemetry {
    private RobotSettings settings = new RobotSettings();
    private Drivetrain drivetrain = settings.drivetrain;
    private Telemetry telemetry;
    private ElapsedTime stateRuntime;
    private ElapsedTime loopTime;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private boolean isEndGame;

    public MainTelemetry(Gamepad gamepadOne, Gamepad gamepadTwo, ElapsedTime runtime, ElapsedTime loopTimer, Telemetry telemetryFromOpMode) {
        stateRuntime = runtime;
        loopTime = loopTimer;
        telemetry = telemetryFromOpMode;
        gamepad1 = gamepadOne;
        gamepad2 = gamepadTwo;
    }
    public void updateTelemetry(){
        endgameCheck();

        double[] powers = drivetrain.getPowers();

        telemetry.addData("Wheel Powers:", "%2.1f, %2.1f, %2.1f, %2.1f", powers[0], powers[1], powers[2], powers[3]);
        telemetry.addData("Loop time:", "%4.2f", loopTime);

        telemetry.update();
    }

    private void endgameCheck(){
        if (stateRuntime.seconds() > 90 && stateRuntime.seconds() < 90.1){
            isEndGame = true;

            gamepad1.rumble(settings.endgameRumbleTime);
            gamepad2.rumble(settings.endgameRumbleTime);

            gamepad1.setLedColor(255.0, 0.0,0.0, 30000);
            gamepad2.setLedColor(255.0, 0.0, 0.0, 30000);
        }
    }
}
