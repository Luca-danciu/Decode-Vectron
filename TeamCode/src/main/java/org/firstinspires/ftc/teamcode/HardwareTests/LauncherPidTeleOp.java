package org.firstinspires.ftc.teamcode.HardwareTests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "Launcher PID TeleOp")


@Configurable
public class LauncherPidTeleOp extends OpMode {

    DcMotorEx launcher;

    // ===== CONFIG =====
    static final double TICKS_PER_REV = 28.0;
    static final double MAX_RPM = 6000.0;

    // PIDF
    public static double kP = 0.00025;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 1.0 / MAX_RPM;

    public static double targetRPM = 5000; // <-- schimbă aici

    // PID state
    double integral = 0;
    double lastError = 0;

    int lastTicks = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");

        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        lastTicks = launcher.getCurrentPosition();
        timer.reset();
    }

    @Override
    public void loop() {

        // ===== TIME =====
        double dt = timer.milliseconds() / 1000;
        timer.reset();
        if (dt <= 0.01) return;

        // ===== RPM CALC =====
        int currentTicks = launcher.getCurrentPosition();
        int deltaTicks = currentTicks - lastTicks;
        lastTicks = currentTicks;

        double currentRPM =
                (deltaTicks / TICKS_PER_REV) / dt * 60.0;

        // ===== PID =====
        double error = targetRPM - currentRPM;
        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double pid = kP * error + kI * integral + kD * derivative;
        double feedforward = kF * targetRPM;

        double power = feedforward + pid;

        // ===== LIMIT =====
        power = Math.max(0, Math.min(1, power));

        launcher.setPower(power);

        lastError = error;

        // ===== TELEMETRY =====
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Power", power);
        telemetry.update();
    }
}

