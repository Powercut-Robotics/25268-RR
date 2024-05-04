package org.firstinspires.ftc.teamcode.powercut;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.powercut.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.powercut.control.PIDController;
import org.firstinspires.ftc.teamcode.powercut.hardware.ArmSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;

public class RobotSettings {
    // DON'T EDIT - DECLARATIONS!
    public Robot robot = new Robot();
    private ArmSystem arm = robot.arm;
    private Drivetrain drivetrain = robot.drivetrain;

    public void init(HardwareMap hardwareMap) {
        robot.init(hardwareMap);
    }


    // PID values and integral limits.
    public PIDCoefficients armPIDCoefficients = new PIDCoefficients(0.0, 0.0, 0.0);
    public PIDCoefficients wristPIDCoefficients = new PIDCoefficients(0.0, 0.0, 0.0);

    public PIDController armPIDController = new PIDController(armPIDCoefficients, arm.armMotor, 0.0);
    public PIDController wristPIDController = new PIDController(wristPIDCoefficients, arm.wristMotor, 0.0);

    // Preset positions
    public int armUpPosition = 1000;
    public int wristUpPosition = 500;

    public int armIntakePosition = 500;
    public int wristIntakePosition = 100;

    public int armDownPosition = 100;
    public int wristDownPosition = 50;

    // Gamepad settings

    public double manualArmControlDeadband = 0.05;
    public double manualWristControlDeadband = 0.05;

    public int endgameRumbleTime = 500;

    // Speed modifiers
    public double totalSpeedModifier = 1.0;
    public double slowmodeSpeedModifier = 0.3;

    // Game-specific modifier
}
