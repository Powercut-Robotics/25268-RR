package org.firstinspires.ftc.teamcode.powercut.control;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDController {
    private DcMotorEx motor;
    private boolean isRunning = false;

    private double P;
    private double I;
    private double D;

    private double integralSumLimit = 0.0;
    private PIDCoefficients pidCoefficientsInternal;

    public PIDController(PIDCoefficients pidCoefficients, DcMotorEx motorToInitalise, Double integralCap) {
        motor = motorToInitalise;

        pidCoefficientsInternal = pidCoefficients;
        P = pidCoefficientsInternal.P;
        I = pidCoefficientsInternal.I;
        D = pidCoefficientsInternal.D;

        integralSumLimit = integralCap;
    }

     public void runTo(Integer target) {
            isRunning = true;
            int error = target - motor.getCurrentPosition();
            int lastError = error;
            double derivative;
            double integralSum = 0;
            double power;
            ElapsedTime loopTime = new ElapsedTime();

            while (Math.abs(error) > 5 && isRunning) {
                error = target - motor.getCurrentPosition();

                // Integral
                integralSum = integralSum + (error * loopTime.seconds());
                // set a limit on our integral sum
                if (integralSum > integralSumLimit) {
                    integralSum = integralSumLimit;
                }
                if (integralSum < -integralSumLimit) {
                    integralSum = -integralSumLimit;
                }

                //Derivative
                derivative = (error - lastError) / loopTime.milliseconds();

                //Get final power
                power = (P * error) + (I * integralSum) + (D * derivative);

                motor.setPower(power);
                //Reset
                lastError = error;
                loopTime.reset();
            }
     }

     public void stop() {
        isRunning = false;
     }
}
