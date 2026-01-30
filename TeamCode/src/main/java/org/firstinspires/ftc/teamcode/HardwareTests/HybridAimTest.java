package org.firstinspires.ftc.teamcode.HardwareTests;

import static org.firstinspires.ftc.teamcode.HardwareTests.AutoAimTurret.TARGET_TAG_ID;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Hardware.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

// presupunem că ai wrapper Limelight
@Disabled
@TeleOp(name = "Hybrid Aim TeleOp", group = "TeleOp")
public class HybridAimTest extends OpMode {

    /* ================== HARDWARE ================== */
    DcMotor turretMotor;
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor RearLeft;
    public DcMotor RearRight;
    Limelight3A limelight;
    Follower follower;

    /* ================== CONSTANTE ================== */
    static final double TARGET_X = 10;
    static final double TARGET_Y = 140;

    static final double TICKS_PER_DEGREE = 6.2;

    static final double kP = 0.005;
    static final double kD = 0.0002;

    static final double AIM_DEADZONE = 0.8; // grade
    static final double MAX_POWER = 0.5;

    static final double MIN_TURRET = -180;
    static final double MAX_TURRET =  180;

    /* ================== PID STATE ================== */
    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    /* ================== INIT ================== */
    @Override
    public void init() {

        RearLeft = hardwareMap.get(DcMotor.class, "RL");
        RearRight = hardwareMap.get(DcMotor.class, "RR");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        FrontLeft = hardwareMap.get(DcMotor.class, "FL");

        turretMotor = hardwareMap.get(DcMotorEx.class, "Tureta");

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

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);

        Pose initialPose = new Pose(35, 84, Math.toRadians(180));
        follower.setPose(initialPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // pipeline AprilTag
        limelight.start();

        timer.reset();
    }

    /* ================== LOOP ================== */
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

        double dt = timer.seconds();
        timer.reset();

        follower.update();

        updateTurretHybridAim(dt);


        telemetry.addData("Turret Error (deg)", lastError);
        telemetry.addData("Limelight tx", getVisionOffset());
        telemetry.update();
    }

    /* ================== HYBRID AIM ================== */
    void updateTurretHybridAim(double dt) {

        double turretTarget =
                normalize(getMapAimAngle() + getVisionOffset());

        double turretCurrent =
                turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

        double error = normalize(turretTarget - turretCurrent);

        if (Math.abs(error) < AIM_DEADZONE) {
            turretMotor.setPower(0);
            lastError = error;
            return;
        }

        double derivative = (error - lastError) / dt;
        double power = kP * error + kD * derivative;

        power = clamp(power, -MAX_POWER, MAX_POWER);

        // 🔒 soft stop mecanic
        if (turretCurrent >= MAX_TURRET && power > 0) power = 0;
        if (turretCurrent <= MIN_TURRET && power < 0) power = 0;

        turretMotor.setPower(power);
        lastError = error;
    }

    /* ================== MAP AIM ================== */
    double getMapAimAngle() {

        Pose pose = follower.getPose();

        double dx = TARGET_X - follower.getPose().getX();
        double dy = TARGET_Y - follower.getPose().getY();

        double absAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = Math.toDegrees(pose.getHeading());

        return normalize(absAngle - robotHeading);
    }

    /* ================== VISION ================== */
    double getVisionOffset() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0.0;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return 0.0;

        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == 20) {
                return tag.getTargetXDegrees();
            }
        }

        return 0.0; // niciun tag valid găsit
    }

    /* ================== UTILS ================== */
    double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
