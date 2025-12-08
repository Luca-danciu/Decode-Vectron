package org.firstinspires.ftc.teamcode.TesteCamera;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Indexer;

// ✅ IMPORTURI PENTRU PANELS
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.PanelsConfigurables;

@TeleOp(name = "Shooter RPM PIDF TeleOp")
@Configurable // ✅ OBLIGATORIU PENTRU PANELS
public class TestTeleOP extends OpMode {

    private DcMotorEx motor;

    // ✅ CPR REAL (28 CPR CU RAPORT 1:2)
    private static final double CPR = 18.666;

    // ✅ PIDF REGLABIL DIN PANELS (OBLIGATORIU public static)
    public static double kP = 0.005;
    public static double kI = 0.000007;
    public static double kD = 0.0001;
    public static double kF = 0.0001468;

    private double integral = 0;
    private double previousError = 0;

    private ElapsedTime timer = new ElapsedTime();
    private double dt = 0.02;

    private int lastEncoder = 0;
    public static double targetRPM = 0; // ✅ ÎL POȚI VEDEA ȘI DIN PANELS

    Indexer indexer = new Indexer();
//    TelemetryManager telem

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotorEx.class, "Launcher");

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        indexer.indexerinit(hardwareMap);

        // ✅ ACTIVARE PANELS CONFIGURABLE
        PanelsConfigurables.INSTANCE.refreshClass(this);

        timer.reset();
    }

    @Override
    public void loop() {

        // ✅ PRESET-URI RPM DIN BUTOANE
        if (gamepad1.square) {
            targetRPM = 3000;
            resetPID();
        }

        if (gamepad1.triangle) {
            targetRPM = 6000;
            resetPID();
        }

        if (gamepad1.circle) {
            targetRPM = 9000;
            resetPID();
        }

        if (gamepad1.cross) {
            targetRPM = 12000;
            resetPID();
        }

        if (gamepad1.ps) {
            targetRPM = 0;
            resetPID();
        }

        // ✅ INDEXER
        if (gamepad1.dpad_up) indexer.Push();
        if (gamepad1.dpad_down) indexer.Down();

        targetRPM = Math.max(0, Math.min(12000, targetRPM));

        // ✅ STOP REAL LA 0 RPM
        if (targetRPM <= 50) {
            motor.setPower(0);
            integral = 0;
            previousError = 0;
            telemetry.addLine("STOP MODE");
            telemetry.update();
            return;
        }

        double currentTime = timer.seconds();

        if (currentTime >= dt) {

            int currentEncoder = motor.getCurrentPosition();
            int delta = currentEncoder - lastEncoder;

            double measuredRPM = (delta * 60.0) / (CPR * currentTime);

            double error = targetRPM - measuredRPM;

            // ✅ DEADZONE ANTI TREPIDAT
            if (Math.abs(error) < 40) error = 0;

            // ✅ INTEGRAL CU LIMITARE (ANTI WIND-UP)
            integral += error * currentTime;
            integral = Math.max(-5000, Math.min(5000, integral));

            // ✅ DERIVATĂ FILTRATĂ
            double derivative = (error - previousError) / Math.max(currentTime, 0.01);

            double output =
                    kP * error +
                            kI * integral +
                            kD * derivative +
                            kF * targetRPM;

            output = Math.max(0.0, Math.min(1.0, output));

            motor.setPower(output);

            previousError = error;
            lastEncoder = currentEncoder;
            timer.reset();

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Measured RPM", measuredRPM);
            telemetry.addData("PWM Output", output);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
    }

    // ✅ RESET PID LA SCHIMBAREA SETPOINT-ULUI
    private void resetPID() {
        integral = 0;
        previousError = 0;
    }
}