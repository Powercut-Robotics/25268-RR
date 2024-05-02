package org.firstinspires.ftc.teamcode.powercut;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSystem {
    private DcMotorEx arm = null;
    private DcMotorEx wrist = null;
    private Servo grip = null;

    public void init(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        wrist = hardwareMap.get(DcMotorEx.class, "gripPose");
        grip = hardwareMap.get(Servo.class, "grip");
    }

    public void gripActivate() {
        grip.setPosition(0.05);
    }

    public void gripRelease() {
        grip.setPosition(0.0);
    }

    public void setArmPower(double armPowerRequested) {
        arm.setPower(armPowerRequested);
    }

    public void setWristPower(double wristPowerRequested) {
        wrist.setPower(wristPowerRequested);
    }
}
