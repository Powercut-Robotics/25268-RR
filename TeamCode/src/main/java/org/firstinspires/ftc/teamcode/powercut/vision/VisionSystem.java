package org.firstinspires.ftc.teamcode.powercut.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.powercut.RobotSettings;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
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

    private final AprilTagLibrary tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();

    public void init(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setLensIntrinsics(658.92, 658.923, 438.245, 202.074)
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
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(aprilTagProcessor, TfodProcessor)
                .build();

        TfodProcessor.setClippingMargins(RobotSettings.clippingMargins[0], RobotSettings.clippingMargins[1], RobotSettings.clippingMargins[2], RobotSettings.clippingMargins[3]);
        // visionSystem.setProcessorEnabled(TfodProcessor, false);
    }

    public List<AprilTagDetection> getAprilTags() {
        aprilTagProcessor.setDecimation(2);

        return aprilTagProcessor.getDetections();
    }

    public List<Recognition> getTfodRecognitions() {
        visionSystem.setProcessorEnabled(TfodProcessor, true);

        List<Recognition> currentRecognitions = TfodProcessor.getRecognitions();

        visionSystem.setProcessorEnabled(TfodProcessor, false);
        return currentRecognitions;
    }

    public boolean isGamepeicePresent() {
        visionSystem.setProcessorEnabled(TfodProcessor, true);

        List<Recognition> currentRecognitions = TfodProcessor.getRecognitions();

        visionSystem.setProcessorEnabled(TfodProcessor, false);

        if (currentRecognitions.isEmpty()) {
            return false;
        } else {
            return true;
        }
    }

    public int getGamepeicePosition() {
        visionSystem.setProcessorEnabled(TfodProcessor, true);

        List<Recognition> currentRecognitions = TfodProcessor.getRecognitions();

        int position = 0;

        for (Recognition recognition : currentRecognitions) {
            float left = recognition.getLeft();
            float right = recognition.getRight();

            if (left >= RobotSettings.spikeMark1[0] && right <= RobotSettings.spikeMark1[1]) {
                position = 1;
            } else if (left >= RobotSettings.spikeMark2[0] && right <= RobotSettings.spikeMark2[1]) {
                position = 2;
            } else if (left >= RobotSettings.spikeMark3[0] && right <= RobotSettings.spikeMark3[1]) {
                position = 3;
            }
        }

        return position;
    }
}