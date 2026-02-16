package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.APRILTAG_PHYSICAL_SIZE_METERS;
import static org.firstinspires.ftc.teamcode.Constants.ID_BT;
import static org.firstinspires.ftc.teamcode.Constants.ID_GPP;
import static org.firstinspires.ftc.teamcode.Constants.ID_PGP;
import static org.firstinspires.ftc.teamcode.Constants.ID_PPG;
import static org.firstinspires.ftc.teamcode.Constants.ID_RT;
import static org.firstinspires.ftc.teamcode.Constants.WEBCAM_RANGE_MODIFIER;
import static org.firstinspires.ftc.teamcode.Constants.WEBCAM_SIZE;


import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

public class WebcamTureta {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;


    private String lastTag = "Niciun tag";
    private double lastDistance = -1;

    public void initwebcam(HardwareMap hardwareMap) {
        AprilTagLibrary myLibrary = new AprilTagLibrary.Builder()
                .addTag(ID_PGP, "PGP", APRILTAG_PHYSICAL_SIZE_METERS, DistanceUnit.METER)
                .addTag(ID_PPG, "PPG", APRILTAG_PHYSICAL_SIZE_METERS, DistanceUnit.METER)
                .addTag(ID_GPP, "GPP", APRILTAG_PHYSICAL_SIZE_METERS, DistanceUnit.METER)
                .addTag(ID_RT, "RT", APRILTAG_PHYSICAL_SIZE_METERS, DistanceUnit.METER)
                .addTag(ID_BT, "BT", APRILTAG_PHYSICAL_SIZE_METERS, DistanceUnit.METER)

                .build();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(myLibrary)
//                .setLensIntrinsics(630.396, 630.396, 318.485, 249.698)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "WebcamTureta"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(WEBCAM_SIZE)
                .build();
    }

    public String detectTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        lastTag = "Niciun tag";
        lastDistance = -1;

        for (AprilTagDetection d : detections) {
            if (d.ftcPose == null) continue;
//  28.26
            if (d.id == ID_PGP) { lastTag = "PGP"; lastDistance = d.ftcPose.range * WEBCAM_RANGE_MODIFIER; break; }
            if (d.id == ID_PPG) { lastTag = "PPG"; lastDistance = d.ftcPose.range * WEBCAM_RANGE_MODIFIER; break; }
            if (d.id == ID_GPP) { lastTag = "GPP"; lastDistance = d.ftcPose.range * WEBCAM_RANGE_MODIFIER; break; }
            if (d.id == ID_BT) { lastTag = "Blue Tower"; lastDistance = d.ftcPose.range * WEBCAM_RANGE_MODIFIER; break; }
            if (d.id == ID_RT) { lastTag = "Red Tower"; lastDistance = d.ftcPose.range * WEBCAM_RANGE_MODIFIER; break; }
        }

        return lastTag;
    }
    public double getTagBearing() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection d : detections) {
            if (d.id == ID_BT && d.ftcPose != null) {
                return d.ftcPose.bearing; // unghiul în grade
            }
        }
        return 0;
    }
    public boolean isTagFound() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.ftcPose != null && d.id == ID_BT) return true;
        }
        return false;
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }
    public double getDistanceMeters() {
        return lastDistance;
    }

    public void close() {
        if (visionPortal != null) visionPortal.close();
    }

    public void startDetection() {
        if (visionPortal != null) visionPortal.resumeStreaming();
    }

    public void stopDetection() {
        if (visionPortal != null) visionPortal.stopStreaming();
    }

    public boolean isDetecting() {
        return visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }
}
