package org.firstinspires.ftc.teamcode.powercut;

import com.qualcomm.robotcore.hardware.Servo;

public class DroneSystem {
    private Servo planeLauncher = null;

    public void init() {
        planeLauncher.setPosition(0.3);
    }

    public void doLaunch() {
        planeLauncher.setPosition(0.8);
    }
}
