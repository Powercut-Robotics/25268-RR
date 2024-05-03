package org.firstinspires.ftc.teamcode.powercut.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.control.PIDController;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;
import org.firstinspires.ftc.teamcode.powercut.teleOp.logic.ArmControlLogic;

@Autonomous
public class MainAutonomous extends OpMode {
    // Declaring the system
    private RobotSettings settings = new RobotSettings();
    private Drivetrain drivetrain = settings.drivetrain;
    public DroneSystem droneSystem = new DroneSystem();

    // Declaring PID controllers
    private PIDController armPIDController = settings.armPIDController;
    private PIDController wristPIDController = settings.wristPIDController;

    // Game monitoring
    private ElapsedTime runtime = new ElapsedTime();

    // System Monitoring
    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

}
