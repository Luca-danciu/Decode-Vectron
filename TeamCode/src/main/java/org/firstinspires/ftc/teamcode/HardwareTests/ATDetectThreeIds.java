package org.firstinspires.ftc.teamcode.HardwareTests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.List;

//@Autonomous(name = "AT Detect 22/23/21", group = "Vision")
public class ATDetectThreeIds extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private static final int ID_PGP = 22;
    private static final int ID_PPG = 23;
    private static final int ID_GPP = 21;

    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagLibrary myLibrary = new AprilTagLibrary.Builder()
                .addTag(ID_PGP, "PGP", 0.16, DistanceUnit.METER)
                .addTag(ID_PPG, "PPG", 0.16, DistanceUnit.METER)
                .addTag(ID_GPP, "GPP", 0.16, DistanceUnit.METER)
                .build();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(myLibrary)
                // valori calibrate pentru Logitech C920
                .setLensIntrinsics(630.396, 630.396, 318.485, 249.698)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)  // 🔹 ADĂUGAT – rezolvă avertismentul
                .setCameraResolution(new Size(1280, 720)) // important pentru calibrare corectă
                .build();

        telemetry.addLine("Init complet. Apasa START pentru detectie...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            String status = "Niciun tag recunoscut";
            Double distanceMeters = null;

            for (AprilTagDetection d : detections) {
                if (d.ftcPose == null) continue;

                if (d.id == ID_PGP) { status = "PGP"; distanceMeters = d.ftcPose.range; break; }
                if (d.id == ID_PPG) { status = "PPG"; distanceMeters = d.ftcPose.range; break; }
                if (d.id == ID_GPP) { status = "GPP"; distanceMeters = d.ftcPose.range; break; }
            }

            telemetry.addData("Status", status);
            if (distanceMeters != null)
                telemetry.addData("Distanta (m)", String.format("%.2f", distanceMeters/28.26));
            telemetry.update();

            sleep(20);
        }

        if (visionPortal != null) visionPortal.close();
    }
}
