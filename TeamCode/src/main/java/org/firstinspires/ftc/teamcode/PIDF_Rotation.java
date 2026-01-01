package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.security.PublicKey;

@Configurable
@TeleOp(name = "Turret PIDF TargetPos Wrap", group = "PID")
public class PIDF_Rotation extends OpMode {

    private DcMotorEx turret;

    // ================= CONFIG =================
    public static final double TICKS_PER_DEGREE = 6.195;
    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREE);

    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0002;
    public static double kF = 0.05;

    // ================= PID =================
    private double integral = 0;
    private double lastError = 0;

    // ================= TARGET =================
    public static int targetPosition = 0;

    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "Tureta");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        // ================= INPUT =================
        if (gamepad1.dpad_up) {
            targetPosition += 20;
        }
        if (gamepad1.dpad_up) {
            targetPosition -= 20;
        }

        // ================= WRAP TARGET =================
        targetPosition = wrapTicks(targetPosition);

        int currentPosition = wrapTicks(turret.getCurrentPosition());

        // ================= PIDF =================
        double power = pidf(targetPosition, currentPosition);
        power = clamp(power, -1, 1);

        turret.setPower(power);

        // ================= TELEMETRY =================
        telemetry.addData("Target ticks", targetPosition);
        telemetry.addData("Current ticks", currentPosition);
        telemetry.addData("Error ticks", targetPosition - currentPosition);
        telemetry.addData("Power", power);
        telemetry.update();
    }

    // ================= PIDF =================
    private double pidf(int target, int current) {

        int error = wrapTicks(target - current);

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return (kP * error)
                + (kI * integral)
                + (kD * derivative)
                + (kF * Math.signum(error));
    }

    // ================= WRAP FUNCTION =================
    private int wrapTicks(int ticks) {
        int range = MAX_TICKS * 2;

        while (ticks > MAX_TICKS) {
            ticks -= range;
        }
        while (ticks < -MAX_TICKS) {
            ticks += range;
        }
        return ticks;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
