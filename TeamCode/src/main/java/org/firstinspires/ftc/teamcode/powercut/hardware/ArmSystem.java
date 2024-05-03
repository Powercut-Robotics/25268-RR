package org.firstinspires.ftc.teamcode.powercut.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSystem {
    public DcMotorEx armMotor = null;
    public DcMotorEx wristMotor = null;
    public Servo grip = null;

    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        wristMotor = hardwareMap.get(DcMotorEx.class, "gripPose");
        grip = hardwareMap.get(Servo.class, "grip");
    }

    public void gripActivate() {
        grip.setPosition(0.05);
    }

    public void gripRelease() {
        grip.setPosition(0.0);
    }

    public void setArmPower(double armPowerRequested) {
        armMotor.setPower(armPowerRequested);
    }

    public void setWristPower(double wristPowerRequested) {
        wristMotor.setPower(wristPowerRequested);
    }

    public void stop() {
        armMotor.setPower(0);
        wristMotor.setPower(0);
    }
}
