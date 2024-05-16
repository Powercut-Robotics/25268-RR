package org.firstinspires.ftc.teamcode.powercut.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


public class VisionSystem {
    public VisionPortal visionSystem;
    private AprilTagProcessor aprilTagProcessor;
    private TfodProcessor TfodProcessor;

    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };

    public void init(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        TfodProcessor = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)

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

    public List<AprilTagDetection> getAprilTags() {
        visionSystem.setProcessorEnabled(aprilTagProcessor, true);
        aprilTagProcessor.setDecimation(2);

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        visionSystem.setProcessorEnabled(aprilTagProcessor, false);
        return currentDetections;
    }

    public List<Recognition> getTfodRecognitions() {
        visionSystem.setProcessorEnabled(TfodProcessor, true);

        List<Recognition> currentRecognitions = TfodProcessor.getRecognitions();

        visionSystem.setProcessorEnabled(TfodProcessor, false);
        return currentRecognitions;
    }
}