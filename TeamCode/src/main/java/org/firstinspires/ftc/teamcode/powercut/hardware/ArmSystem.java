package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;



public class ArmSystem {
    public DcMotorEx armMotor = null;
    public DcMotorEx wristMotor = null;
    public Servo gripLeft = null;
    public Servo gripRight = null;
    public TouchSensor armResetTouchSensor = null;
    public TouchSensor wristResetTouchSensor = null;


    protected PIDEx armPID = new PIDEx(RobotSettings.armDownCoefficients);
    protected PIDEx wristPID = new PIDEx(RobotSettings.wristCoefficients);


    // resets and inits
    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        wristMotor = hardwareMap.get(DcMotorEx.class, "gripPose");
        gripLeft = hardwareMap.get(Servo.class, "gripLeft");
        gripRight = hardwareMap.get(Servo.class, "gripRight");
        armResetTouchSensor = hardwareMap.get(TouchSensor.class, "armResetSensor");
        wristResetTouchSensor = hardwareMap.get(TouchSensor.class, "wristResetSensor");


        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }


    public void resetEncoders() {
        DcMotorEx.RunMode armMode = armMotor.getMode();
        DcMotorEx.RunMode wristMode = wristMotor.getMode();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(armMode);
        wristMotor.setMode(wristMode);
    }

    public void resetArmEncoder() {
        DcMotor.RunMode armMode = armMotor.getMode();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(armMode);
    }

    public void resetWristEncoder() {
        DcMotor.RunMode wristMode = wristMotor.getMode();
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(wristMode);
    }

    // powers and grip
    public void gripLeftActivate() {
        gripLeft.setPosition(0.09);
    }
    public void gripRightActivate() {
        gripRight.setPosition(0.09);
    }

    public void gripLeftRelease() {
        gripLeft.setPosition(0.15);
    }
    public void gripRightRelease() {
        gripRight.setPosition(0.15);
    }

    public void gripLeftTuck() { gripLeft.setPosition(0.5); }
    public void gripRightTuck() { gripRight.setPosition(0.5); }

    public void setArmPower(double armPowerRequested) {

        if (armResetTouchSensor.isPressed()) {
            resetArmEncoder();
        }

        if (armResetTouchSensor.isPressed() && armPowerRequested > 0) {
            armMotor.setPower(0);
        } else if (Math.abs(armMotor.getCurrentPosition()) > Math.abs(RobotSettings.armLimit) && armPowerRequested < 0) {
            armMotor.setPower(0);
        } else {
            armMotor.setPower(armPowerRequested);
        }

    }

    public void setWristPower(double wristPowerRequested) {
        if (wristResetTouchSensor.isPressed()) {
            resetWristEncoder();
        }

        if ((wristPowerRequested > 0) && wristResetTouchSensor.isPressed()) {
            wristMotor.setPower(0);
        } else {
            wristMotor.setPower(wristPowerRequested);
        }
    }

    public void doPresetArm() {
        boolean isReset = armResetTouchSensor.isPressed();

        while (!isReset) {
            setArmPower(RobotSettings.armPresetSpeed);
            isReset = armResetTouchSensor.isPressed();
        }

        resetArmEncoder();
        armMotor.setPower(0);
    }

    public void doPresetWrist() {
        boolean isReset = wristResetTouchSensor.isPressed();

        while (!isReset) {
            setWristPower(RobotSettings.wristPresetSpeed);
            isReset = wristResetTouchSensor.isPressed();
        }

        resetWristEncoder();
        wristMotor.setPower(0);
    }


    //BIG STUFF
    public class ArmUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            double armTarget = RobotSettings.armUpPosition;
            packet.put("armTarget", armTarget);

            double armPos = armMotor.getCurrentPosition();
            packet.put("armPos", armPos);

            double armPower;
            armPower = armPID.calculate(armTarget, armPos);
            packet.put("armPower", armPower);

            setArmPower(armPower);


            if ((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - RobotSettings.armDeadband) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + RobotSettings.armDeadband)) {
                armMotor.setPower(0);
                packet.put("Arm complete", 1);
                return false;
            } else {
                return true;
            }


        }
    }

    public Action armUp() {
        return new ArmUp();
    }

    public class WristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            double wristTarget = RobotSettings.wristUpPosition;
            packet.put("wristTarget", wristTarget);

            double wristPos = wristMotor.getCurrentPosition();
            packet.put("wristPos", wristPos);

            double wristPower;
            wristPower = wristPID.calculate(wristTarget, wristPos);
            packet.put("wristPower", wristPower);

            setWristPower(wristPower);

            if ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - RobotSettings.wristDeadband) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + RobotSettings.wristDeadband)) {
                wristMotor.setPower(0);
                packet.put("Wrist complete", 1);
                return false;
            } else {
                return true;
            }

        }
    }

    public Action wristUp() {
        return new WristUp();
    }

    public class ArmDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (armResetTouchSensor.isPressed()) {
                resetArmEncoder();
                return false;
            } else {
                double armTarget = RobotSettings.armDownPosition;
                packet.put("armTarget", armTarget);

                double armPos = armMotor.getCurrentPosition();
                packet.put("armPos", armPos);

                double armPower;
                armPower = armPID.calculate(armTarget, armPos);
                packet.put("armPower", armPower);

                setArmPower(armPower);

                if ((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - RobotSettings.armDeadband) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + RobotSettings.armDeadband)) {
                    armMotor.setPower(0);
                    return false;
                } else {
                    return true;
                }
            }
        }
    }

    public Action armDown() {
        return new ArmDown();
    }

    public class WristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            double wristTarget = RobotSettings.wristDownPosition;
            packet.put("wristTarget", wristTarget);

            double wristPos = wristMotor.getCurrentPosition();
            packet.put("wristPos", wristPos);

            double wristPower;
            wristPower = wristPID.calculate(wristTarget, wristPos);
            packet.put("wristPower", wristPower);

            setWristPower(wristPower);

            if ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - RobotSettings.wristDeadband) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + RobotSettings.wristDeadband)) {
                wristMotor.setPower(0);
                return false;
            } else {
                return true;
            }

        }
    }

    public Action wristDown() {
        return new WristDown();

    }

    public class PresetArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean isReset = armResetTouchSensor.isPressed();

            if (isReset) {
                armMotor.setPower(0);
                resetArmEncoder();
                return false;
            } else {
                setArmPower(RobotSettings.armPresetSpeed);
                return true;
            }

        }
    }

    public Action presetArm() {
        return new PresetArm();
    }

    public class PresetWrist implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean isReset = wristResetTouchSensor.isPressed();

            if (isReset) {
                wristMotor.setPower(0);
                resetWristEncoder();
                return false;
            } else {
                setWristPower(RobotSettings.wristPresetSpeed);
                return true;
            }

        }
    }

    public Action presetWrist() {
        return new PresetWrist();
    }
    // set grip

    public class GripLeftActivate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripLeftActivate();
            return false;
        }
    }

    public Action gripLeftActivateAction() {
        return new GripLeftActivate();
    }




    public class GripRightActivate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripRightActivate();
            return false;
        }
    }

    public Action gripRightActivateAction() {
        return new GripRightActivate();
    }



    public class GripLeftRelease implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripLeftRelease();
            return false;
        }
    }

    public Action gripLeftReleaseAction() {
        return new GripLeftRelease();
    }



    public class GripRightRelease implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripRightRelease();
            return false;
        }
    }

    public Action gripRightReleaseAction() {
        return new GripRightRelease();
    }



    public class GripLeftTuck implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripLeftTuck();
            return false;
        }
    }

    public Action gripLeftTuckAction() {
        return new GripLeftTuck();
    }


    public class GripRightTuck implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripRightTuck();
            return false;
        }
    }

    public Action gripRightTuckAction() {
        return new GripRightTuck();
    }

    public class GripActivate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripLeftActivate();
            gripRightActivate();
            return false;
        }
    }

    public Action gripActivate() {
        return new GripActivate();
    }

    public class GripRelease implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripLeftRelease();
            gripRightRelease();
            return false;
        }
    }

    public Action gripRelease() {
        return new GripRelease();
    }

    public class GripTuck implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripLeftTuck();
            gripRightTuck();
            return false;
        }
    }

    public Action gripTuck() {
        return new GripTuck();
    }

    public void stop() {
        armMotor.setPower(0);
        wristMotor.setPower(0);
    }
}
