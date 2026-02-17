package org.firstinspires.ftc.teamcode.Hardware;


import static org.firstinspires.ftc.teamcode.OpmodesConstants.APRILTAG_FIELD_XY;
import static org.firstinspires.ftc.teamcode.OpmodesConstants.ID_BT;
import static org.firstinspires.ftc.teamcode.OpmodesConstants.ID_RT;
import static org.firstinspires.ftc.teamcode.OpmodesConstants.LIMELIGHT_APRILTAG_PIPELINE;
import static org.firstinspires.ftc.teamcode.OpmodesConstants.LIMELIGHT_CAM_OFFSET_FORWARD_IN;
import static org.firstinspires.ftc.teamcode.OpmodesConstants.LIMELIGHT_CAM_OFFSET_LATERAL_IN;
import static org.firstinspires.ftc.teamcode.OpmodesConstants.LIMELIGHT_DISTANCE_DIVISOR;
import static org.firstinspires.ftc.teamcode.OpmodesConstants.LIMELIGHT_DISTANCE_SCALE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpmodesConstants;

import java.util.List;
@Configurable
public class Limelight {
    private static final String TAG = "Limelight";

    private static double PITCH = 0;
    private static double ROLL = 0;

    public Limelight3A limelight;
    public IMU imu;
    double distance;

    /** Debug: why getFieldPoseFromAprilTag returned null (or "OK" when valid). */
    public String poseDebugReason = "";
    /** Debug: tag IDs seen, e.g. "20,24". */
    public String poseDebugTagsSeen = "";
    /** Debug: raw botpose x,y in meters before conversion. */
    public String poseDebugRawXY = "";

    public void limelightinit(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_APRILTAG_PIPELINE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public String getAprilTag() {

        limelight.pipelineSwitch(LIMELIGHT_APRILTAG_PIPELINE);

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return "NONE";
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return "NONE";
        }

        // Only care about tower tags 20 (Blue) and 24 (Red); ignore others
        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (id == ID_BT) return "BT";
            if (id == ID_RT) return "RT";
        }
        return "NONE";
    }

    public double getAprilTagDistance() {

        // IMU
        // YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            if (fiducials != null && hasTowerTag(fiducials)) {
                distance = LIMELIGHT_DISTANCE_SCALE / llResult.getTa();
            }
        }
        return distance / LIMELIGHT_DISTANCE_DIVISOR;

    }

    /**
     * Returns the robot's field pose from AprilTag vision (MegaTag2) when a valid
     * tower tag (20 or 24) is detected. Used to correct odometry drift during TeleOp.
     * Only tags 20 (Blue) and 24 (Red) are used; others are ignored. Tag positions
     * come from {@link OpmodesConstants#APRILTAG_FIELD_XY}.
     * <p>
     * Requires Limelight web UI: robot-space pose configured, field map uploaded,
     * and "Full 3D" enabled in AprilTag pipeline (Advanced tab).
     *
     * @return Field pose (x, y in inches; heading in radians), or null if no valid tower tag seen
     */
    public Pose getFieldPoseFromAprilTag(Pose pose) {
        poseDebugReason = "";
        poseDebugTagsSeen = "";
        poseDebugRawXY = "";

//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        PanelsTelemetry.INSTANCE.getTelemetry().addData("YAW", pose.getHeading());

        YawPitchRollAngles orientation = new YawPitchRollAngles(AngleUnit.RADIANS, pose.getHeading(), 0, 0, 0);
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) {
            poseDebugReason = "result null or invalid";
            PanelsTelemetry.INSTANCE.getTelemetry().addData(TAG, "getFieldPose: " + poseDebugReason);
            return null;
        }

        // Botpose is only valid when at least one AprilTag is in view
        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            poseDebugReason = "no fiducials (no AprilTag in view)";
            PanelsTelemetry.INSTANCE.getTelemetry().addData(TAG, "getFieldPose: " + poseDebugReason);
            return null;
        }

        // Filter to only tags 20 and 24 (tower tags); ignore others
        fiducials = filterTowerTags(fiducials);
        if (fiducials.isEmpty()) {
            poseDebugReason = "no tower tags (20 or 24) in view, only others";
            PanelsTelemetry.INSTANCE.getTelemetry().addData(TAG, "getFieldPose: " + poseDebugReason);
            return null;
        }

        StringBuilder tagIds = new StringBuilder();
        for (int i = 0; i < fiducials.size(); i++) {
            if (i > 0) tagIds.append(",");
            tagIds.append(fiducials.get(i).getFiducialId());
        }
        poseDebugTagsSeen = tagIds.toString();

        // Try MegaTag2 first (IMU-fused); fall back to MegaTag1 if MT2 returns zeros
        Pose3D botPose = llResult.getBotpose_MT2();
        boolean usedMT1 = false;
        if (botPose != null) {
            double xM = botPose.getPosition().x;
            double yM = botPose.getPosition().y;
            poseDebugRawXY = String.format("MT2 x=%.3f y=%.3f m", xM, yM);
            if (Math.abs(xM) < 0.01 && Math.abs(yM) < 0.01) {
                PanelsTelemetry.INSTANCE.getTelemetry().addData(TAG, "getFieldPose: MT2 returned zeros, tags=" + poseDebugTagsSeen + ", trying MT1");
                botPose = null;
            }
        }
        if (botPose == null) {
            botPose = llResult.getBotpose();
            usedMT1 = true;
        }
        if (botPose == null) {
            poseDebugReason = "botPose MT2+MT1 both null (check Full 3D + field map in Limelight UI)";
            PanelsTelemetry.INSTANCE.getTelemetry().addData(TAG, "getFieldPose: " + poseDebugReason + ", tags=" + poseDebugTagsSeen);
            return null;
        }

        if (usedMT1) {
            poseDebugRawXY = String.format("MT1 x=%.3f y=%.3f m", botPose.getPosition().x, botPose.getPosition().y);
        }

        // Limelight returns position in meters; convert to inches to match odometry/field coords
        double xInches = botPose.getPosition().x * 39.3701;
        double yInches = botPose.getPosition().y * 39.3701;
        double headingRad = botPose.getOrientation().getYaw(AngleUnit.RADIANS);

        // Reject (0,0) poses - fall back to manual compute from first tag
        if (Math.abs(xInches) < 0.1 && Math.abs(yInches) < 0.1) {
            Pose fallback = computePoseFromFirstTag(fiducials);
            if (fallback != null) {
                poseDebugReason = "OK (fallback from tag)";
                poseDebugRawXY = String.format("tag %s manual", poseDebugTagsSeen);
                PanelsTelemetry.INSTANCE.getTelemetry().addData(TAG, "getFieldPose: " + poseDebugReason + " pose=(" + String.format("%.1f", fallback.getX()) + "," + String.format("%.1f", fallback.getY()) + ") in");
                return fallback;
            }
            poseDebugReason = "pose 0,0 + no tag pos (configure Limelight field map or add tag to APRILTAG_FIELD_XY)";
            PanelsTelemetry.INSTANCE.getTelemetry().addData(TAG, "getFieldPose: " + poseDebugReason + ", tags=" + poseDebugTagsSeen);
            return null;
        }

        poseDebugReason = "OK";
        PanelsTelemetry.INSTANCE.getTelemetry().addData(TAG, "getFieldPose: OK tags=" + poseDebugTagsSeen + " pose=(" + String.format("%.1f", xInches) + "," + String.format("%.1f", yInches) + ") in");
        return new Pose(xInches, yInches, headingRad);
    }

    /** Returns true if any fiducial is tower tag (20 or 24). */
    private boolean hasTowerTag(List<LLResultTypes.FiducialResult> fiducials) {
        if (fiducials == null) return false;
        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (id == ID_BT || id == ID_RT) return true;
        }
        return false;
    }

    /** Filters fiducials to only include tower tags 20 and 24. */
    private List<LLResultTypes.FiducialResult> filterTowerTags(List<LLResultTypes.FiducialResult> fiducials) {
        if (fiducials == null) return java.util.Collections.emptyList();
        return fiducials.stream()
                .filter(f -> f.getFiducialId() == ID_BT || f.getFiducialId() == ID_RT)
                .collect(java.util.stream.Collectors.toList());
    }

    /**
     * Maps tag ID to APRILTAG_FIELD_XY index. Returns -1 if not a tower tag.
     * Index 0 = tag 20, index 1 = tag 24.
     */
    private int tagIdToFieldIdx(int tagId) {
        if (tagId == ID_BT) return 0;
        if (tagId == ID_RT) return 1;
        return -1;
    }

    /**
     * Fallback when botpose returns zeros: compute robot pose from first tag using
     * getCameraPoseTargetSpace() and known tag positions from Constants.
     */
    private Pose computePoseFromFirstTag(List<LLResultTypes.FiducialResult> fiducials) {
        if (fiducials == null || fiducials.isEmpty()) return null;

        // Use first tower tag (20 or 24)
        LLResultTypes.FiducialResult tag = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == ID_BT || f.getFiducialId() == ID_RT) {
                tag = f;
                break;
            }
        }
        if (tag == null) return null;

        int tagId = tag.getFiducialId();
        int idx = tagIdToFieldIdx(tagId);
        if (idx < 0 || idx >= APRILTAG_FIELD_XY.length) return null;

        double tagX = APRILTAG_FIELD_XY[idx][0];
        double tagY = APRILTAG_FIELD_XY[idx][1];

        Pose3D camToTag = tag.getCameraPoseTargetSpace();
        if (camToTag == null) return null;

        // Limelight: x=lateral, z=forward (in tag frame). Convert m -> in.
        double xCam = camToTag.getPosition().x * 39.3701;
        double zCam = camToTag.getPosition().z * 39.3701;

        // Robot offset from tag (camera is on robot)
        double xRobot = zCam + LIMELIGHT_CAM_OFFSET_FORWARD_IN;
        double yRobot = -xCam + LIMELIGHT_CAM_OFFSET_LATERAL_IN;

        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Robot offset in field coords
        double fieldDx = xRobot * Math.cos(heading) - yRobot * Math.sin(heading);
        double fieldDy = xRobot * Math.sin(heading) + yRobot * Math.cos(heading);

        double robotX = tagX - fieldDx;
        double robotY = tagY - fieldDy;

        return new Pose(robotX, robotY, heading);
    }
}