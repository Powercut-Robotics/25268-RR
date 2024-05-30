package org.firstinspires.ftc.teamcode.powercut;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotSettings {
    // PID values and integral limits.

    public static PIDCoefficientsEx armUpCoefficients = new PIDCoefficientsEx(0.001, 0.001, 0.001, 25000, 0, 0);
    public static PIDCoefficientsEx armDownCoefficients = new PIDCoefficientsEx(0.001, 0.001, 0.001, 25000, 0, 0);
    public static PIDCoefficientsEx wristCoefficients = new PIDCoefficientsEx(0.001, 0, 0, 25.0, 1.0, 0.5);

    public static double armDeadband = 5;
    public static double wristDeadband = 5;

    public static double armUpperLimit = -4000;
    public static double armLowerLimit = 200;

    // Preset positions
    public static int armUpPosition = -4000;
    public static int  wristUpPosition = -30;


    public static int armDownPosition = 0;
    public static int wristDownPosition = -100;

    // Gamepad settings

    public static double manualArmControlDeadband = 0.005;
    public static double manualWristControlDeadband = 0.005;

    public int endgameRumbleTime = 200;

    // Speed modifiers
    public static double totalSpeedModifier = 1.0;
    public static double slowmodeSpeedModifier = 0.2;

    // Game-specific modifiers
    public static float[] spikeMark1 = {0, 200};
    public static float[] spikeMark2 = {0, 640};
    public static float[] spikeMark3 = {450, 640};

    //vision modifiers
    public static int[] clippingMargins = {0,0,0,0};

}
