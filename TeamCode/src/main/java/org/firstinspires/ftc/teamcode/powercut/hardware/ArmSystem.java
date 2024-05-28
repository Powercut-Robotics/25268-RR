package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;



public class ArmSystem {
    public DcMotorEx armMotor = null;
    public DcMotorEx wristMotor = null;
    public Servo gripLeft = null;
    public Servo gripRight = null;

    private final RobotSettings settings = new RobotSettings();

    private PIDEx armUpPID = new PIDEx(RobotSettings.armUpCoefficients);
    private PIDEx armDownPID = new PIDEx(RobotSettings.armDownCoefficients);
    private PIDEx wristPID = new PIDEx(RobotSettings.wristCoefficients);

    
    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        wristMotor = hardwareMap.get(DcMotorEx.class, "gripPose");
        gripLeft = hardwareMap.get(Servo.class, "gripLeft");
        gripRight = hardwareMap.get(Servo.class, "gripRight");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



    }

    public void gripLeftActivate() {
        gripLeft.setPosition(0.09);
    }
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


    public void gripRightActivate() {
        gripRight.setPosition(0.09);
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


    public void gripLeftRelease() {
        gripLeft.setPosition(0.15);
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


    public void gripRightRelease() {
        gripRight.setPosition(0.15);
    }
    public class GripRightRelease implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripRightRelease();
            return false;
        }
    }
    public Action gripRightReleaseAction() {
        return new GripLeftRelease();
    }

    public void gripLeftTuck() { gripLeft.setPosition(0.5); }

    public void gripRightTuck() { gripRight.setPosition(0.5); }

    public void setArmPower(double armPowerRequested) {
        if ((Math.abs(armMotor.getCurrentPosition()) > Math.abs(RobotSettings.armUpperLimit)) && armPowerRequested < 0) {
            armMotor.setPower(0);
        } else if ((armMotor.getCurrentPosition() > RobotSettings.armLowerLimit) && armPowerRequested > 0) {
            armMotor.setPower(0);
        } else {
            armMotor.setPower(armPowerRequested);
        }

    }

    public void setWristPower(double wristPowerRequested) {
        wristMotor.setPower(wristPowerRequested);
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
    public class ArmUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double armTarget = RobotSettings.armUpPosition;
            double wristTarget = RobotSettings.wristUpPosition;

            packet.put("armTarget", armTarget);
            packet.put("wristTarget", wristTarget);

            // checks lift's current position
            double armPos = armMotor.getCurrentPosition();
            double wristPos = wristMotor.getCurrentPosition();

            packet.put("armPos", armPos);
            packet.put("wristPos", wristPos);

            double armPower;
            double wristPower;

            armPower = armUpPID.calculate(armTarget, armPos);
            wristPower = wristPID.calculate(wristTarget, wristPos);

            packet.put("armPower", armPower);
            packet.put("wristPower", wristPower);
            
            armMotor.setPower(armPower);
            wristMotor.setPower(wristPower);
            
            if (((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - RobotSettings.armDeadband) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + RobotSettings.armDeadband)) && ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - RobotSettings.wristDeadband) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + RobotSettings.wristDeadband))) {
                armMotor.setPower(0);
                wristMotor.setPower(0);
                return false;
            } else  {
                // true causes the action to rerun
                return true;
            }
        }
    }

    public Action armUp() {
        return new ArmUp();
    }

    public class ArmDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            double armTarget = RobotSettings.armDownPosition;
            double wristTarget = RobotSettings.wristDownPosition;

            packet.put("armTarget", armTarget);
            packet.put("wristTarget", wristTarget);
            
            // checks lift's current position
            double armPos = armMotor.getCurrentPosition();
            double wristPos = wristMotor.getCurrentPosition();
            
            packet.put("armPos", armPos);
            packet.put("wristPos", wristPos);
            
            double armPower;
            double wristPower;

            armPower = armDownPID.calculate(armTarget, armPos);
            wristPower = wristPID.calculate(wristTarget, wristPos);

            packet.put("armPower", armPower);
            packet.put("wristPower", wristPower);


            armMotor.setPower(armPower);
            wristMotor.setPower(wristPower);

            if (((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - RobotSettings.armDeadband) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + RobotSettings.armDeadband)) && ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - RobotSettings.wristDeadband) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + RobotSettings.wristDeadband))) {
                armMotor.setPower(0);
                wristMotor.setPower(0);
                return false;
            } else  {
                // true causes the action to rerun
                return true;
            }
        }
    }

    public Action armDown() {
        return new ArmDown();
    }

    public void resetEncoders() {
        DcMotorEx.RunMode armMode = armMotor.getMode();
        DcMotorEx.RunMode wristMode = wristMotor.getMode();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(armMode);
        wristMotor.setMode(wristMode);
    }
    
    public void stop() {
        armMotor.setPower(0);
        wristMotor.setPower(0);
    }
}
