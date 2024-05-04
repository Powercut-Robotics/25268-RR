package org.firstinspires.ftc.teamcode.powercut.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public ArmSystem arm = new ArmSystem();
    public Drivetrain drivetrain = new Drivetrain();
    public DroneSystem droneSystem = new DroneSystem();

    public void init(HardwareMap hardwareMap) {
        arm.init(hardwareMap);
        drivetrain.init(hardwareMap);
        droneSystem.init(hardwareMap);
    }

    public void stop() {
        drivetrain.stop();
        arm.stop();
    }
}
