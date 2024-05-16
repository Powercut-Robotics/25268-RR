package org.firstinspires.ftc.teamcode.powercut.hardware;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
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
