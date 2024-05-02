package org.firstinspires.ftc.teamcode.powercut;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.sql.Array;

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

     public double[] getPowers(){
        double[] powers = {0.0,0.0,0.0,0.0};
        powers[0] = frontLeft.getPower();
        powers[1] = frontRight.getPower();
        powers[2] = backLeft.getPower();
        powers[3] = backRight.getPower();

        return powers;
     }
}
