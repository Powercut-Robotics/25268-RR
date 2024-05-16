package org.firstinspires.ftc.teamcode.powercut.control;

public class PIDCoefficients {
    public double P;
    public double I;
    public double D;

    public PIDCoefficients(Double proportional, Double integral, Double derivative) {
        P = proportional;
        I = integral;
        D = derivative;
    }
}
