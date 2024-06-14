package org.firstinspires.ftc.teamcode.powercut;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotSettings {
    // PID values and integral limits.
    public static PIDCoefficientsEx armCoefficients = new PIDCoefficientsEx(0.0015, 0.001, 0.001, 25000, 0, 0);
    public static PIDCoefficientsEx wristCoefficients = new PIDCoefficientsEx(0.025, 0, 0, 25.0, 1.0, 0.5);

    public static double armDeadband = 5;
    public static double wristDeadband = 2;

    public static double armLimit = -3600;

    public static double armPresetSpeed = 1.0;
    public static double wristPresetSpeed = 0.5;

    public static double wristSpeedModifier = 0.25;

    // Preset positions
    public static int armUpPosition = -3450;
    public static int  wristUpPosition = -70;


    public static int armDownPosition = 50;
    public static int wristDownPosition = -106;

    // Gamepad settings

    public static double manualArmControlDeadband = 0.005;
    public static double manualWristControlDeadband = 0.005;

    public int endgameRumbleTime = 500;

    // Speed modifiers
    public static double totalSpeedModifier = 1.0;
    public static double slowmodeSpeedModifier = 0.2;
}
