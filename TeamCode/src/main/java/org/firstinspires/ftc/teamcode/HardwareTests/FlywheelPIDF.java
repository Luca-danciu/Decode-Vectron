package org.firstinspires.ftc.teamcode.HardwareTests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp(name="Flywheel PIDF")
public class FlywheelPIDF extends LinearOpMode {

    DcMotorEx flywheel;

    public static double P = 30;
    public static double I = 0;
    public static double D = 2;
    public static double F = 12;

    static final double MAX_TICKS_PER_SEC = 2800;

    double targetVelocity = 0;

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "Launcher");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            flywheel.setVelocityPIDFCoefficients(P, I, D, F);

            if (gamepad1.dpad_up) {
                targetVelocity = MAX_TICKS_PER_SEC;
            }

            if (gamepad1.dpad_left) {
                targetVelocity = MAX_TICKS_PER_SEC - 100;
            }

            if (gamepad1.dpad_right) {
                targetVelocity = MAX_TICKS_PER_SEC + 100;
            }

            if (gamepad1.dpad_down) {
                targetVelocity = 0;
            }

            flywheel.setVelocity(targetVelocity);

            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Actual", flywheel.getVelocity());
            telemetry.addData("Error", targetVelocity - flywheel.getVelocity());
            telemetry.addData("P", P);
            telemetry.addData("I", I);
            telemetry.addData("D", D);
            telemetry.addData("F", F);
            telemetry.update();
        }
    }
}
