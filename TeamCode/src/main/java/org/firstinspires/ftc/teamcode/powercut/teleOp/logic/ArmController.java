package org.firstinspires.ftc.teamcode.powercut.teleOp.logic;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.powercut.teleOp.MainTeleOp;
import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.control.PIDController;
import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;

public class ArmController extends MainTeleOp {
    private RobotSettings settings = new RobotSettings();
    private ArmSystem arm = robot.arm;
//    public ArmController(HardwareMap hardwareMap) {
//        arm.init(hardwareMap);
//    }

    private PIDController armPIDController = settings.armPIDController;
    private PIDController wristPIDController = settings.wristPIDController;

    public void doArmControl() {
        double armSpeed = gamepad2.right_trigger - gamepad2.left_trigger;
        double wristSpeed = gamepad2.right_stick_y;

        if (Math.abs(armSpeed) > settings.manualArmControlDeadband || Math.abs(wristSpeed) > settings.manualWristControlDeadband) {
            armPIDController.stop();
            wristPIDController.stop();

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
            armPIDController.runTo(settings.armUpPosition);
            wristPIDController.runTo(settings.wristUpPosition);
        } else if (gamepad2.circle) {
            armPIDController.runTo(settings.armIntakePosition);
            wristPIDController.runTo(settings.wristIntakePosition);
        } else if (gamepad2.cross) {
            armPIDController.runTo(settings.armDownPosition);
            wristPIDController.runTo(settings.wristDownPosition);
        } else if (gamepad2.square) {
            //
        }
    }
}
