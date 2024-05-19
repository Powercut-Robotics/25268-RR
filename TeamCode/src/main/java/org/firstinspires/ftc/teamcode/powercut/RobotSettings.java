package org.firstinspires.ftc.teamcode.powercut;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotSettings {
    // PID values and integral limits.
    public PIDCoefficientsEx armCoefficients = new PIDCoefficientsEx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public PIDCoefficientsEx wristCoefficients = new PIDCoefficientsEx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);


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
