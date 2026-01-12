package org.firstinspires.ftc.teamcode.HardwareTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled

@TeleOp
public class LimeLightAprilTagTest extends OpMode {
    public Limelight3A limelight;
    public IMU imu;
    double distance;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    @Override
    public void start(){
        limelight.start();
    }
    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid() && llResult != null){
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());

            telemetry.addData("CalculatedDist" , distance/10000);
            telemetry.addData("Tx" , llResult.getTx());
//            telemetry.addData("Ty" , llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose" , botPose.toString());
//            telemetry.addData("orientation ", botPose.getOrientation().getYaw());
        }
        telemetry.update();
    }
    public double getDistanceFromTag(double ta){
        double scale = 28197.31;
        double distance = (scale/ ta);
        return distance;
    }
}
