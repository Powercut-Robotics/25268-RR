package org.firstinspires.ftc.teamcode.powercut;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

import java.lang.reflect.Array;

@Config
public class RobotSettings {
    // PID values and integral limits.

    public static PIDCoefficientsEx armCoefficients = new PIDCoefficientsEx(0.04, 0.01, 0.03, 25, 100, 0.3);
    public static PIDCoefficientsEx wristCoefficients = new PIDCoefficientsEx(0.001, 0.01, 0.2, 25.0, 1.0, 0.5);

    public static double armDeadband = 5;
    public static double wristDeadband = 5;

    public static double armUpperLimit = -4000;
    public static double armLowerLimit = 200;

    // Preset positions
    public static int armUpPosition = -3355;
    public static int  wristUpPosition = 5;

    public static int armIntakePosition = -102;
    public static int wristIntakePosition = -80;

    public static int armDownPosition = -102;
    public static int wristDownPosition = -90;

    // Gamepad settings

    public static double manualArmControlDeadband = 0.005;
    public static double manualWristControlDeadband = 0.005;

    public int endgameRumbleTime = 500;

    // Speed modifiers
    public static double totalSpeedModifier = 1.0;
    public static double slowmodeSpeedModifier = 0.3;

    // Game-specific modifiers
    public static float[] spikeMark1 = {0, 150};
    public static float[] spikeMark2 = {150, 450};
    public static float[] spikeMark3 = {450, 640};
}
