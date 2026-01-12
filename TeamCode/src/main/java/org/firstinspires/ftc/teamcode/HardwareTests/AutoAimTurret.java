package org.firstinspires.ftc.teamcode.HardwareTests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.List;

@TeleOp(name = "AutoAim Turret PIDF (Ticks)", group = "Vision")
public class AutoAimTurret extends OpMode {

    // ================= HARDWARE =================

    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor RearLeft;
    public DcMotor RearRight;
    private DcMotorEx turret;
    private Limelight3A limelight;

    // ================= TURETA CONFIG =================
    static final double TICKS_PER_DEGREE = 6.195;

    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREE);
    public static final int MIN_TICKS = -MAX_TICKS;

    // ================= PIDF =================
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0002;
    public static double kF = 0.05;

    private double integral = 0;
    private double lastError = 0;
    static final int TARGET_TAG_ID = 20;

    // ================= CONTROL =================
    private boolean targetVisible = false;
     int desiredTicks = 0;

    @Override
    public void init() {

        RearLeft = hardwareMap.get(DcMotor.class, "RL");
        RearRight = hardwareMap.get(DcMotor.class, "RR");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        FrontLeft = hardwareMap.get(DcMotor.class, "FL");

        turret = hardwareMap.get(DcMotorEx.class, "Tureta");

        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        RearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // pipeline AprilTag
        limelight.start();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)));
        FrontLeft.setPower((forward + strafe + turn) / denominator * 2);
        RearLeft.setPower((forward - (strafe - turn)) / denominator * 2);
        FrontRight.setPower((forward - (strafe + turn)) / denominator * 2);
        RearRight.setPower((forward + (strafe - turn)) / denominator * 2);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            turret.setPower(0);
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            turret.setPower(0);
            return;
        }

// caută ID-ul dorit
        LLResultTypes.FiducialResult targetTag = null;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == TARGET_TAG_ID) {
                targetTag = tag;
                break;
            }
            else {
                telemetry.addData("Tag" , tag);
            }
        }

        if (targetTag == null) {
            turret.setPower(0);
            telemetry.addData("Status", "Wait for TAG");
            return;
        }

// ===== AUTO AIM =====
        double tx = targetTag.getTargetXDegrees();

        if (Math.abs(tx) < 1.0) {
            turret.setPower(0);
            return;
        }

        double kAim = 0.03;
        double power = tx * kAim;

        if (Math.abs(power) < 0.1) {
            power = 0.1 * Math.signum(power);
        }

        power = Math.max(-0.6, Math.min(0.6, power));

// limite mecanice
        int pos = turret.getCurrentPosition();
        if ((pos >= MAX_TICKS && power > 0) ||
                (pos <= MIN_TICKS && power < 0)) {
            power = 0;
        }

        turret.setPower(power);

        telemetry.addData("Tracking Tag", TARGET_TAG_ID);
        telemetry.addData("TX", tx);
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

    // ================= WRAP =================
    private int wrapTicks(int ticks) {
        int range = MAX_TICKS * 2;

        while (ticks > MAX_TICKS) ticks -= range;
        while (ticks < -MAX_TICKS) ticks += range;

        return ticks;
    }

    private void stopTurret() {
        turret.setPower(0);
        integral = 0;
        lastError = 0;
        targetVisible = false;
    }

    @Override
    public void stop() {
        turret.setPower(0);
        limelight.stop();
    }
}
