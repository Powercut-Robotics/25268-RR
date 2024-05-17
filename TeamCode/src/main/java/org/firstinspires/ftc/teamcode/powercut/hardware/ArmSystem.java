package org.firstinspires.ftc.teamcode.powercut.hardware;

import androidx.annotation.NonNull;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.control.PIDCoefficients;
import org.firstinspires.ftc.teamcode.powercut.control.PIDController;

import java.util.function.DoubleSupplier;


public class ArmSystem {
    public DcMotorEx armMotor = null;
    public DcMotorEx wristMotor = null;
    public Servo grip = null;

    private final RobotSettings settings = new RobotSettings();

    private PIDEx armPID = new PIDEx(settings.armCoefficients);

    private PIDEx wristPID = new PIDEx(settings.wristCoefficients);

    private PIDCoefficients wristPIDCoefficients = settings.wristPIDCoefficients;


    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        wristMotor = hardwareMap.get(DcMotorEx.class, "gripPose");
        grip = hardwareMap.get(Servo.class, "grip");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gripActivate() {
        grip.setPosition(0.05);
    }

    public void gripRelease() {
        grip.setPosition(0.0);
    }

    public void setArmPower(double armPowerRequested) {
        armMotor.setPower(armPowerRequested);
    }

    public void setWristPower(double wristPowerRequested) {
        wristMotor.setPower(wristPowerRequested);
    }

    public class ArmUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double target = 5000;
            // checks lift's current position
            double pos = armMotor.getCurrentPosition();
            packet.put("armPos", pos);
            double armPower;

            armPower = armPID.calculate(target, pos);
            armMotor.setPower(armPower);



            if ((Math.abs(armMotor.getCurrentPosition()) > Math.abs(target) - 5) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(target) + 5)) {
                armMotor.setPower(0);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            } else  {
                // true causes the action to rerun
                return true;
            }
        }
    }

    public Action armUp() {
        return new ArmUp();
    }

    public class ArmIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double target = 2000;
            // checks lift's current position
            double pos = armMotor.getCurrentPosition();
            packet.put("armPos", pos);
            double armPower;

            armPower = armPID.calculate(target, pos);
            armMotor.setPower(armPower);

            if ((Math.abs(armMotor.getCurrentPosition()) > Math.abs(target) - 5) && (Math.abs(armMotor.getCurrentPosition()) < Math.abs(target) + 5)) {
                armMotor.setPower(0);
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return false;
            } else  {
                // true causes the action to rerun
                return true;
            }
        }
    }

    public Action armIntake() {
        return new ArmIntake();
    }

    public void stop() {
        armMotor.setPower(0);
        wristMotor.setPower(0);
    }
}
