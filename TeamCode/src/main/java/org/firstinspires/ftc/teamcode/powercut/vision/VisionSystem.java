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
import java.util.function.DoubleSupplier;


public class VisionSystem {
    public VisionPortal visionSystem;
    private AprilTagProcessor aprilTagProcessor;
    public ColourMassDetectionProcessor colourMassDetectionProcessor;


    private final AprilTagLibrary tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();

    DoubleSupplier minAreaSupplier = () -> RobotSettings.minArea;
    DoubleSupplier leftLineSupplier = () -> RobotSettings.leftLine;
    DoubleSupplier rightLineSupplier = () -> RobotSettings.rightLine;

    public void init(HardwareMap hardwareMap) {
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setTagLibrary(tagLibrary)
//                .setLensIntrinsics(658.92, 658.923, 438.245, 202.074)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .build();


        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                RobotSettings.lowerHSV,
                RobotSettings.upperHSV,
                minAreaSupplier,
                leftLineSupplier,
                rightLineSupplier
        );

        visionSystem = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                //.setCameraResolution(new Size(640, 480))
                //.enableLiveView(true)
                //.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(colourMassDetectionProcessor)
                .build();
    }

//    public List<AprilTagDetection> getAprilTags() {
//        aprilTagProcessor.setDecimation(2);
//
//        return aprilTagProcessor.getDetections();
//    }



}