package org.firstinspires.ftc.teamcode.powercut.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain {
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
    private IMU imu = null;

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        imu.resetYaw();

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

     public void setDrivetrainPowers(Double frontLeftRequest, Double frontRightRequest, Double backLeftRequest, Double backRightRequest) {
        frontLeft.setPower(frontLeftRequest);
        frontRight.setPower(frontRightRequest);
        backLeft.setPower(backLeftRequest);
        backRight.setPower(backRightRequest);
     }

    public void doPowerFromGamepad(Double axial, Double lateral, Double yaw, Double modifier) {
        // Math out the power for each wheel
        double frontLeftPowerCalculated = (axial + lateral + yaw) * modifier;
        double frontRightPowerCalculated = (axial - lateral - yaw) * modifier;
        double backLeftPowerCalculated = (axial - lateral + yaw) * modifier;
        double backRightPowerCalculated = (axial + lateral - yaw) * modifier;

        this.setDrivetrainPowers(frontLeftPowerCalculated, frontRightPowerCalculated, backLeftPowerCalculated, backRightPowerCalculated);
    }
    public double getHeading() {
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        return robotOrientation.getYaw(AngleUnit.DEGREES);
    }

     public double[] getPowers(){
        double[] powers = {0.0,0.0,0.0,0.0};
        powers[0] = frontLeft.getPower();
        powers[1] = frontRight.getPower();
        powers[2] = backLeft.getPower();
        powers[3] = backRight.getPower();

        return powers;
     }

     public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
     }

}
