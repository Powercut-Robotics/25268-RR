package org.firstinspires.ftc.teamcode.powercut.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.teamcode.powercut.control.PIDController;
import org.firstinspires.ftc.teamcode.powercut.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.powercut.hardware.DroneSystem;
import org.firstinspires.ftc.teamcode.powercut.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class MainAutonomous extends OpMode {
    // Declaring the system
    private RobotSettings settings = new RobotSettings();
    private Robot robot = settings.robot;
    private Drivetrain drivetrain = robot.drivetrain;
    private DroneSystem droneSystem = robot.droneSystem;

    // Declaring PID controllers
    private PIDController wristPIDController = settings.wristPIDController;

    // Game monitoring
    private ElapsedTime runtime = new ElapsedTime();

    // System Monitoring
    private ElapsedTime loopTime = new ElapsedTime();

    private int state = 0;

    private Pose2d start = new Pose2d(0,0,0);
    private MecanumDrive roadrunnerDrivetrain = new MecanumDrive(hardwareMap, start);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {


        updateTelemetry();
        loopTime.reset();
    }

    public void updateTelemetry() {
        double[] powers = drivetrain.getPowers();

        telemetry.addData("Wheel Powers:", "%2.1f, %2.1f, %2.1f, %2.1f", powers[0], powers[1], powers[2], powers[3]);
        telemetry.addData("Loop time:", "%4.2f", loopTime.milliseconds());

        telemetry.update();
    }
}
