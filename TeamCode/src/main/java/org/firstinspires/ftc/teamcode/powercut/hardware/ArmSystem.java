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

    private PIDEx armPID = new PIDEx(RobotSettings.armCoefficients);

    private PIDEx wristPID = new PIDEx(RobotSettings.wristCoefficients);

    
    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        wristMotor = hardwareMap.get(DcMotorEx.class, "gripPose");
        gripLeft = hardwareMap.get(Servo.class, "gripLeft");
        gripRight = hardwareMap.get(Servo.class, "gripRight");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



    }

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

    public void armToPosition(double target) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = armMotor.getCurrentPosition();
        double armPower = 0;

        while ((Math.abs(armMotor.getCurrentPosition()) > Math.abs(target) - RobotSettings.armDeadband) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(target) + RobotSettings.armDeadband)) {
            currentPosition = armMotor.getCurrentPosition();
            armPower = armPID.calculate(target, currentPosition);
            armMotor.setPower(armPower);
        }

        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void wristToPosition(double target) {
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = wristMotor.getCurrentPosition();
        double wristPower = 0;

        while ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(target) - RobotSettings.wristDeadband) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(target) + RobotSettings.wristDeadband)) {
            wristPower = wristPID.calculate(target, currentPosition);
            wristMotor.setPower(wristPower);
        }

        wristMotor.setPower(0);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double armTarget = RobotSettings.armUpPosition;
            double wristTarget = RobotSettings.wristUpPosition;
            
            // checks lift's current position
            double armPos = armMotor.getCurrentPosition();
            double wristPos = wristMotor.getCurrentPosition();
            
            packet.put("armPos", armPos);
            packet.put("wristPos", wristPos);

            double armPower;
            double wristPower;
            
            armPower = armPID.calculate(armTarget, armPos);
            wristPower = wristPID.calculate(wristTarget, wristPos);
            
            armMotor.setPower(armPower);
            wristMotor.setPower(wristPower);
            
            if (((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - RobotSettings.armDeadband) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + RobotSettings.armDeadband)) && ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - RobotSettings.wristDeadband) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + RobotSettings.wristDeadband))) {
                armMotor.setPower(0);
                wristMotor.setPower(0);
                
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public class ArmIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double armTarget = RobotSettings.armIntakePosition;
            double wristTarget = RobotSettings.wristIntakePosition;
            
            // checks lift's current position
            double armPos = armMotor.getCurrentPosition();
            double wristPos = wristMotor.getCurrentPosition();
            
            packet.put("armPos", armPos);
            packet.put("wristPos", wristPos);
            
            double armPower;
            double wristPower;
            
            armPower = armPID.calculate(armTarget, armPos);
            wristPower = wristPID.calculate(wristTarget, wristPos);
            
            armMotor.setPower(armPower);
            wristMotor.setPower(wristPower);
            
            if (((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - RobotSettings.armDeadband) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + RobotSettings.armDeadband)) && ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - RobotSettings.wristDeadband) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + RobotSettings.wristDeadband))) {
                armMotor.setPower(0);
                wristMotor.setPower(0);
                
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            } else  {
                // true causes the action to rerun
                return true;
            }
        }
    }

    public Action armIntake() {
        return new ArmIntake();
    }

        public class ArmDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double armTarget = RobotSettings.armDownPosition;
            double wristTarget = RobotSettings.wristDownPosition;
            
            // checks lift's current position
            double armPos = armMotor.getCurrentPosition();
            double wristPos = wristMotor.getCurrentPosition();
            
            packet.put("armPos", armPos);
            packet.put("wristPos", wristPos);
            
            double armPower;
            double wristPower;
            
            armPower = armPID.calculate(armTarget, armPos);
            wristPower = wristPID.calculate(wristTarget, wristPos);
            
            armMotor.setPower(armPower);
            wristMotor.setPower(wristPower);

            if (((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - RobotSettings.armDeadband) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + RobotSettings.armDeadband)) && ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - RobotSettings.wristDeadband) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + RobotSettings.wristDeadband))) {
                armMotor.setPower(0);
                wristMotor.setPower(0);
                
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    
    public void stop() {
        armMotor.setPower(0);
        wristMotor.setPower(0);
    }
}
