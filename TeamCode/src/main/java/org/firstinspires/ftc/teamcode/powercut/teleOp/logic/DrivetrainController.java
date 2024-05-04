package org.firstinspires.ftc.teamcode.powercut.teleOp.logic;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;

public class DrivetrainController {
    private RobotSettings settings = new RobotSettings();
    private Robot robot = settings.robot;
    private Drivetrain drivetrain = robot.drivetrain;
    public void doPowerFromGamepad(Double axial, Double lateral, Double yaw, Double modifier) {
        // Math out the power for each wheel
        double frontLeftPowerCalculated = (axial + lateral + yaw) * modifier;
        double frontRightPowerCalculated = (axial - lateral - yaw) * modifier;
        double backLeftPowerCalculated = (axial - lateral + yaw) * modifier;
        double backRightPowerCalculated = (axial + lateral - yaw) * modifier;

        drivetrain.setDrivetrainPowers(frontLeftPowerCalculated, frontRightPowerCalculated, backLeftPowerCalculated, backRightPowerCalculated);
    }
}
