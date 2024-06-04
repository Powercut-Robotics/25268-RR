package org.firstinspires.ftc.teamcode.powercut.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.hardware.ArmActions;
import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;
import org.firstinspires.ftc.teamcode.powercut.vision.VisionSystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp
public class Test extends OpMode {

    private MecanumDrive drive;
    private VisionSystem visionSystem = new VisionSystem();
    private ArmSystem arm = new ArmSystem();
    private ArmActions armActions = new ArmActions();
    private DroneSystem droneSystem = new DroneSystem();
    @Override
    public void init() {
        visionSystem.init(hardwareMap);
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
    public void loop() {
        double armSpeed = -gamepad2.left_stick_y;
        double wristSpeed = gamepad2.right_stick_y;

        if (Math.abs(armSpeed) > RobotSettings.manualArmControlDeadband || Math.abs(wristSpeed) > RobotSettings.manualWristControlDeadband) {
            arm.setArmPower(armSpeed);
            arm.setWristPower(wristSpeed * 0.5);
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

        telemetry.addData("Arm Reset Button", arm.armResetTouchSensor.isPressed());
        telemetry.addData("Wrist Reset Button", arm.armResetTouchSensor.isPressed());
    }

}
