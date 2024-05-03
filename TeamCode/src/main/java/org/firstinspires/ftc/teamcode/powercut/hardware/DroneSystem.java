package org.firstinspires.ftc.teamcode.powercut.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneSystem {
    private Servo planeLauncher = null;

    public void init(HardwareMap hardwareMap) {
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
    }

    public void preset() {
        planeLauncher.setPosition(0.3);
    }
    public void doLaunch() {
        planeLauncher.setPosition(0.8);
    }
}
