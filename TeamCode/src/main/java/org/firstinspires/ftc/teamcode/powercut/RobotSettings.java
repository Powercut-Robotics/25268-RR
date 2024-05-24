package org.firstinspires.ftc.teamcode.powercut;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

import java.lang.reflect.Array;


public class RobotSettings {
    // PID values and integral limits.

    public PIDCoefficientsEx armCoefficients = new PIDCoefficientsEx(0.3, 0.05, 0.2, 100.0, 0.0, 0.2);
    public PIDCoefficientsEx wristCoefficients = new PIDCoefficientsEx(0.02, 0.001, 0.1, 0.0, 0.0, 0.0);


    // Preset positions
    public int armUpPosition = -3387;
    public int wristUpPosition = 0;;

    public int armIntakePosition = -645;
    public int wristIntakePosition = -60;

    public int armDownPosition = -0;
    public int wristDownPosition = -44;

    // Gamepad settings

    public double manualArmControlDeadband = 0.01;
    public double manualWristControlDeadband = 0.01;

    public int endgameRumbleTime = 500;

    // Speed modifiers
    public double totalSpeedModifier = 1.0;
    public double slowmodeSpeedModifier = 0.3;

    // Game-specific modifiers
    public float[] spikeMark1 = {0, 150};
    public float[] spikeMark2 = {150, 450};
    public float[] spikeMark3 = {450, 640};
}
