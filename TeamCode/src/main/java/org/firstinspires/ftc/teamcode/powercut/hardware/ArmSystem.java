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

    private PIDEx armPID = new PIDEx(settings.armCoefficients);

    private PIDEx wristPID = new PIDEx(settings.wristCoefficients);

    
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

    public void setArmPower(double armPowerRequested) {
        armMotor.setPower(armPowerRequested);
    }

    public void setWristPower(double wristPowerRequested) {
        wristMotor.setPower(wristPowerRequested);
    }

    public void armToPosition(double target) {
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = armMotor.getCurrentPosition();
        double armPower = 0;

        while ((Math.abs(armMotor.getCurrentPosition()) > Math.abs(target) - 5) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(target) + 5)) {
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

        while ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(target) - 5) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(target) + 5)) {
            wristPower = wristPID.calculate(target, currentPosition);
            wristMotor.setPower(wristPower);
        }

        wristMotor.setPower(0);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public class ArmUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double armTarget = settings.armUpPosition;
            double wristTarget = settings.wristUpPosition;
            
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
            
            if (((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - 5) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + 5)) && ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - 5) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + 5))) {
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
            double armTarget = settings.armIntakePosition;
            double wristTarget = settings.wristIntakePosition;
            
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
            
            if (((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - 5) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + 5)) && ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - 5) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + 5))) {
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
            double armTarget = settings.armDownPosition;
            double wristTarget = settings.wristDownPosition;
            
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
            
            if (((Math.abs(armMotor.getCurrentPosition()) > Math.abs(armTarget) - 5) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(armTarget) + 5)) && ((Math.abs(wristMotor.getCurrentPosition()) > Math.abs(wristTarget) - 5) && (Math.abs(wristMotor.getCurrentPosition()) < Math.abs(wristTarget) + 5))) {
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
