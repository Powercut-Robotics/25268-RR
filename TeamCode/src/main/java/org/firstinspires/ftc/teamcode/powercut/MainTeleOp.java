package org.firstinspires.ftc.teamcode.powercut;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Drive")
public class MainTeleOp extends OpMode {
    // Declaring the system
    private Drivetrain drivetrain = new Drivetrain();
    private ArmSystem arm = new ArmSystem();
    private DroneSystem droneSystem = new DroneSystem();

    //Game monitoring
    private boolean isEndGame = false;
    private ElapsedTime runtime = new ElapsedTime();

    // System Monitoring
    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        droneSystem.init();
    }

    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
        doMecanumPowersAndAssign();
        armControl();
        droneControl();

        // Telemetry
        updateTelemetry();
        loopTime.reset();
    }

    private void armControl() {
        double armSpeed = gamepad2.right_trigger - gamepad2.left_trigger;
        double wristSpeed = gamepad2.right_stick_y;

        arm.setArmPower(armSpeed);
        arm.setWristPower(wristSpeed);
    }

    private void droneControl() {
        if (gamepad1.triangle && isEndGame) {
            droneSystem.doLaunch();
        }
    }

    private void doMecanumPowersAndAssign(){
        double modifier = getSpeedModifier();
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Math out the power for each wheel
        double frontLeftPowerCalculated = (axial + lateral + yaw) * modifier;
        double frontRightPowerCalculated = (axial - lateral - yaw) * modifier;
        double backLeftPowerCalculated = (axial - lateral + yaw) * modifier;
        double backRightPowerCalculated = (axial + lateral - yaw) * modifier;

        // Send the calculated powers to the drivetrain
        drivetrain.setDrivetrainPowers(frontLeftPowerCalculated, frontRightPowerCalculated, backLeftPowerCalculated, backRightPowerCalculated);
    }

    private double getSpeedModifier(){
        double speedMultiplier = 1.0;
        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            speedMultiplier = 0.3;
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

            gamepad1.rumble(500);
            gamepad2.rumble(500);

            gamepad1.setLedColor(255.0, 0.0,0.0, 30000);
            gamepad2.setLedColor(255.0, 0.0, 0.0, 30000);
        }
    }


}
