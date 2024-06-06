package org.firstinspires.ftc.teamcode.powercut;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class RobotSettings {
    // PID values and integral limits.

    public static PIDCoefficientsEx armUpCoefficients = new PIDCoefficientsEx(0.0015, 0.001, 0.001, 25000, 0, 0);
    public static PIDCoefficientsEx armDownCoefficients = new PIDCoefficientsEx(0.0015, 0.001, 0.001, 25000, 0, 0);
    public static PIDCoefficientsEx wristCoefficients = new PIDCoefficientsEx(0.01, 0, 0, 25.0, 1.0, 0.5);

    public static double armDeadband = 10;
    public static double wristDeadband = 2;

    public static double armLimit = -4000;

    public static double armPresetSpeed = 0.75;
    public static double wristPresetSpeed = 0.5;

    // Preset positions
    public static int armUpPosition = -3500;
    public static int  wristUpPosition = 0;


    public static int armDownPosition = 0;
    public static int wristDownPosition = -90;

    // Gamepad settings

    public static double manualArmControlDeadband = 0.005;
    public static double manualWristControlDeadband = 0.005;

    public int endgameRumbleTime = 200;

    // Speed modifiers
    public static double totalSpeedModifier = 1.0;
    public static double slowmodeSpeedModifier = 0.2;

    //vision modifiers
    // Game-specific modifiers

    public static Scalar lowerHSV = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
    public static Scalar upperHSV = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
    public static double minArea = 100;

    public static double leftLine = 213;

    public static double rightLine = 426;



}
