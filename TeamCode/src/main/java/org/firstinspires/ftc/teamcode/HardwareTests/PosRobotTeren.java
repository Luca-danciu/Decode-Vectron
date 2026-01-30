package org.firstinspires.ftc.teamcode.HardwareTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

//@TeleOp(name = "AprilTag Localization Map", group = "Test")
public class PosRobotTeren extends LinearOpMode {

    Limelight3A limelight;
    IMU imu;

    // POZITIA TAG-ULUI PE TEREN (INCH)
    static final double TAG_X = 16;
    static final double TAG_Y = 130;

    // OFFSET CAMERA -> CENTRU ROBOT (INCH)
    static final double CAM_OFFSET_FORWARD = 4.0; // camera in fata robotului
    static final double CAM_OFFSET_LATERAL = 0.0; // camera pe centru

    // Dimensiunea mini-map
    static final int MAP_SIZE = 20;
    static final double MAP_SCALE = 5.0; // inch per celula

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        limelight.pipelineSwitch(2);
        limelight.start();

        telemetry.addLine("Ready - point camera at AprilTag");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                LLResultTypes.FiducialResult targetTag = null;

                for (LLResultTypes.FiducialResult t : tags) {
                    if (t.getFiducialId() == 20) {
                        targetTag = t;
                        break;
                    }
                }

                if (targetTag != null) {

                    Pose3D camToTag = targetTag.getCameraPoseTargetSpace();

                    // Limelight -> metri -> inch
                    double xCam = camToTag.getPosition().x * 39.3701; // lateral
                    double zCam = camToTag.getPosition().z * 39.3701; // in fata

                    // Transformare Limelight -> Robot
                    double xRobot = zCam + CAM_OFFSET_FORWARD;  // in fata
                    double yRobot = -xCam + CAM_OFFSET_LATERAL; // lateral (+ stanga, - dreapta)

                    // Heading robot
                    double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    // Transformare Robot -> Field
                    double fieldDx = xRobot * Math.cos(heading) - yRobot * Math.sin(heading);
                    double fieldDy = xRobot * Math.sin(heading) + yRobot * Math.cos(heading);

                    double robotX = TAG_X - fieldDx;
                    double robotY = TAG_Y - fieldDy;

                    // Telemetrie numerică
                    telemetry.addData("Robot X (inch)", robotX);
                    telemetry.addData("Robot Y (inch)", robotY);
                    telemetry.addData("Heading (deg)", Math.toDegrees(heading));

                    // ==== Mini-map ASCII ====
                    char[][] map = new char[MAP_SIZE][MAP_SIZE];
                    for (int i = 0; i < MAP_SIZE; i++)
                        for (int j = 0; j < MAP_SIZE; j++)
                            map[i][j] = '.';

                    // Tag pe hartă
                    int tagCol = (int)(TAG_X / MAP_SCALE);
                    int tagRow = MAP_SIZE - 1 - (int)(TAG_Y / MAP_SCALE); // invertim Y pt vizualizare
                    if (tagRow >= 0 && tagRow < MAP_SIZE && tagCol >= 0 && tagCol < MAP_SIZE)
                        map[tagRow][tagCol] = 'T';

                    // Robot pe hartă
                    int robotCol = (int)(robotX / MAP_SCALE);
                    int robotRow = MAP_SIZE - 1 - (int)(robotY / MAP_SCALE);
                    if (robotRow >= 0 && robotRow < MAP_SIZE && robotCol >= 0 && robotCol < MAP_SIZE)
                        map[robotRow][robotCol] = 'R';

                    // Construim string-ul map
                    StringBuilder mapStr = new StringBuilder();
                    for (int i = 0; i < MAP_SIZE; i++) {
                        for (int j = 0; j < MAP_SIZE; j++) {
                            mapStr.append(map[i][j]);
                        }
                        mapStr.append('\n');
                    }

                    telemetry.addLine("Mini-map (T=Tag, R=Robot):");
                    telemetry.addLine(mapStr.toString());

                } else {
                    telemetry.addLine("Tag 20 not visible");
                }

            } else {
                telemetry.addLine("No valid Limelight result");
            }

            telemetry.update();
        }
    }
}
