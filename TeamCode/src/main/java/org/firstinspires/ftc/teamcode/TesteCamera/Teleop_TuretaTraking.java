package org.firstinspires.ftc.teamcode.TesteCamera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// ----------------------------
//   TELEOP CU TURRETA TRACKING
// ----------------------------

@TeleOp(name = "Field Tracking TeleOp", group = "Pedro")
public class Teleop_TuretaTraking extends OpMode {

    private Follower follower;        // Pedro odometry
    private Outtake outtake = new Outtake();;          // subsistem unde e motorul turetei

    // ----------------------------
    //   Pozitia de start
    // ----------------------------
    private final Pose startPose = new Pose(
            70,
            -70,
            Math.toRadians(0)
    );

    // ----------------------------
    //   Coordonatele turnului
    // ----------------------------
    private static final double towerX = -66;
    private static final double towerY = -183;

    // ----------------------------
    //   Variabile pentru tureta
    // ----------------------------
    private boolean urmarireActivata = false;
    private static final double TICKS_PER_DEGREE = 10.5;   // Pune VALOAREA TA REALĂ

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        outtake.outtakeinit(hardwareMap);

        telemetry.addLine("TeleOp Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // -------------------------------------------------------
        //   Update la Pedro ca să ai poziția exactă în timp real
        // -------------------------------------------------------
        follower.update();

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = Math.toDegrees(follower.getPose().getHeading());


        // -------------------------------------------------------
        //   Control activare / dezactivare turta tracking
        // -------------------------------------------------------
        if (gamepad1.dpad_left) {
            urmarireActivata = true;
        }

        if (gamepad1.dpad_right) {
            urmarireActivata = false;
            outtake.turetaMotor.setTargetPosition(0);
            outtake.turetaMotor.setPower(1);
        }


        // -------------------------------------------------------
        //   TURRETA TRACKING CĂTRE TURN
        // -------------------------------------------------------
        if (urmarireActivata) {

            // unghi absolut către turn
            double targetAngleDeg = Math.toDegrees(
                    Math.atan2(towerY - robotY, towerX - robotX)
            );

            // unghi relativ (față de robot)
            double angleRelative = targetAngleDeg + robotHeading;
            angleRelative = AngleUnit.normalizeDegrees(angleRelative);

            // limite mechanice ±180°
            angleRelative = Range.clip(angleRelative, -180, 180);

            // convertire în ticks
            int targetTicks = (int)(angleRelative * TICKS_PER_DEGREE);

            outtake.turetaMotor.setTargetPosition(targetTicks);
            outtake.turetaMotor.setPower(1);
        }


        // --------------------------
        //  TELEMETRY UTILĂ
        // --------------------------
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading°", robotHeading);

        telemetry.addData("Turret Tracking", urmarireActivata);
        telemetry.addData("Tower Angle°", Math.toDegrees(Math.atan2(towerY - robotY, towerX - robotX)));

        telemetry.update();
    }

}


