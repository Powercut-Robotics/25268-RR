package org.firstinspires.ftc.teamcode.powercut.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


public class visionPortal {
    public VisionPortal visionSystem;
    private AprilTagProcessor aprilTagProcessor;
    TfodProcessor TfodProcessor;
    public void init(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        TfodProcessor = new TfodProcessor.Builder()
                .build();

        visionSystem = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(aprilTagProcessor, TfodProcessor)
                .build();

        visionSystem.setProcessorEnabled(aprilTagProcessor, false);
        visionSystem.setProcessorEnabled(TfodProcessor, false);
    }

    public void getAprilTags() {

    }
}