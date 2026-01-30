package org.firstinspires.ftc.teamcode.HardwareTests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@TeleOp
public class TuretaTestUltim extends OpMode {
    // În clasa ta de OpMode
    private Follower follower;
    private TurretSubsystem turret;

    private final Pose START_POSE = new Pose(35.0, 84, Math.toRadians(0)); // Exemplu: poziție start + heading 90° (spre nord)

    // Exemplu target fix (ajustează după terenul tău, ex. un hang zone)
    private final double TARGET_X = 12.0; // inches
    private final double TARGET_Y = 144.0;

    @Override
    public void init() {
        // Inițializează Follower-ul Pedro Pathing normal
        follower = Constants.createFollower(hardwareMap);

        // Setează starting pose (critic pentru odometrie corectă!)
        follower.setStartingPose(START_POSE);

        // Inițializează turela
        turret = new TurretSubsystem(hardwareMap, follower, TARGET_X, TARGET_Y, START_POSE);
    }

    @Override
    public void loop() {
        follower.update(); // Obligatoriu! Updatează odometria Pinpoint

        turret.update(); // Update turela

        // Restul codului tău (drive, etc.)
        telemetry.addData("Robot Pose", "x=%.1f y=%.1f h=%.1f°",
                follower.getPose().getX(), follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Turret Angle", "%.1f°", turret.getCurrentTurretAngle());
        telemetry.update();
    }
}
