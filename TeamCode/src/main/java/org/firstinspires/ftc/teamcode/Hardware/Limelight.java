package org.firstinspires.ftc.teamcode.Hardware;

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
        limelight.pipelineSwitch(8);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public String getAprilTag() {

        limelight.pipelineSwitch(8);

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
            case 20:
                return "BT";
            case 21:
                return "GPP";
            case 22:
                return "PGP";
            case 23:
                return "PPG";
            case 24:
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
            distance = 28197.31 / llResult.getTa();
        }
        return distance / 10000;

    }
}