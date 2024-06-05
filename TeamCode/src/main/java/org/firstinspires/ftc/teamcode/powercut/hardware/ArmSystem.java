package org.firstinspires.ftc.teamcode.powercut.hardware;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public DigitalChannel armLimitSensor = null;

    protected PIDEx armPID = new PIDEx(RobotSettings.armCoefficients);
    protected PIDEx wristPID = new PIDEx(RobotSettings.wristCoefficients);


    // resets and inits
    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        wristMotor = hardwareMap.get(DcMotorEx.class, "gripPose");
        gripLeft = hardwareMap.get(Servo.class, "gripLeft");
        gripRight = hardwareMap.get(Servo.class, "gripRight");
        armResetTouchSensor = hardwareMap.get(TouchSensor.class, "armResetSensor");
        wristResetTouchSensor = hardwareMap.get(TouchSensor.class, "wristResetSensor");
        armLimitSensor = hardwareMap.get(DigitalChannel.class, "armLimitSensor");

        armLimitSensor.setMode(DigitalChannel.Mode.INPUT);


        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



    }

    protected boolean armLimitIsReached() {
        return armLimitSensor.getState();
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
        DcMotorEx.RunMode armMode = armMotor.getMode();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(armMode);
    }

    public void resetWristEncoder() {
        DcMotorEx.RunMode wristMode = wristMotor.getMode();
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
        } else if (armLimitIsReached() && armPowerRequested < 0) {
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
            armMotor.setPower(RobotSettings.armPresetSpeed);
            isReset = armResetTouchSensor.isPressed();
        }

        armMotor.setPower(0);
    }

    public void doPresetWrist() {
        boolean isReset = wristResetTouchSensor.isPressed();

        while (!isReset) {
            wristMotor.setPower(RobotSettings.wristPresetSpeed);
            isReset = wristResetTouchSensor.isPressed();
        }

        wristMotor.setPower(0);
    }

    public void stop() {
        armMotor.setPower(0);
        wristMotor.setPower(0);
    }
}
