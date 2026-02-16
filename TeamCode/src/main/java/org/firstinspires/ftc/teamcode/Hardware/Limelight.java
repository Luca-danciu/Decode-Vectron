package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.ID_BT;
import static org.firstinspires.ftc.teamcode.Constants.ID_GPP;
import static org.firstinspires.ftc.teamcode.Constants.ID_PGP;
import static org.firstinspires.ftc.teamcode.Constants.ID_PPG;
import static org.firstinspires.ftc.teamcode.Constants.ID_RT;
import static org.firstinspires.ftc.teamcode.Constants.LIMELIGHT_APRILTAG_PIPELINE;
import static org.firstinspires.ftc.teamcode.Constants.LIMELIGHT_DISTANCE_DIVISOR;
import static org.firstinspires.ftc.teamcode.Constants.LIMELIGHT_DISTANCE_SCALE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Limelight {
    public Limelight3A limelight;
    public IMU imu;
    double distance;

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

        int id = fiducials.get(0).getFiducialId();

        switch (id) {
            case ID_BT:
                return "BT";
            case ID_GPP:
                return "GPP";
            case ID_PGP:
                return "PGP";
            case ID_PPG:
                return "PPG";
            case ID_RT:
                return "RT";
            default:
                return "No case detected";
        }
    }

    public double getAprilTagDistance() {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid() && llResult != null) {
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = LIMELIGHT_DISTANCE_SCALE / llResult.getTa();
        }
        return distance / LIMELIGHT_DISTANCE_DIVISOR;

    }
}