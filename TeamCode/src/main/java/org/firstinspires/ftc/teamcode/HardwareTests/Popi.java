package org.firstinspires.ftc.teamcode.HardwareTests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Configurable
@TeleOp
public class Popi extends LinearOpMode {

    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor RearLeft;
    public DcMotor RearRight;
    private Follower follower;

    public int towerX = 5;
    public int towerY = 140;

    private DcMotorEx turret;
    static final double TICKS_PER_DEGREE = 6.071111;

    double dx = 0;
    double dy = 0;
    double headingRobot = 0;
    double alpha = 0;
    double headingTuretaGrade;
    double targetTicks = 0;
    double power;
    private double integral = 0;
    private double lastError = 0;
    public double tan = 0;

    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0002;
    public static double kF = 0.05;
    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREE);
//    static final int START_OFFSET_TICKS = (int)(180 * TICKS_PER_DEGREE);

    public static final int MIN_TICKS = -MAX_TICKS;
    private final Pose START_POSE = new Pose(35.0, 84, Math.toRadians(180)); // Exemplu: poziție start + heading 90° (spre nord)

    @Override
    public void runOpMode() throws InterruptedException {
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
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose( START_POSE );

        waitForStart();

        while (opModeIsActive()){
            follower.update();

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)));
            FrontLeft.setPower((forward + strafe + turn) / denominator * 2);
            RearLeft.setPower((forward - (strafe - turn)) / denominator * 2);
            FrontRight.setPower((forward - (strafe + turn)) / denominator * 2);
            RearRight.setPower((forward + (strafe - turn)) / denominator * 2);


            double dx = towerX - follower.getPose().getX();
            double dy = towerY - follower.getPose().getY();

            double alpha1 = Math.toDegrees(Math.atan2(dy, dx));
            double headingRobot = Math.toDegrees(follower.getHeading());

            double headingTuretaGrade = alpha1 - headingRobot;
            if (headingTuretaGrade > 180){
                headingTuretaGrade = headingTuretaGrade - 360;
            }
            if (headingTuretaGrade < -180){
                headingTuretaGrade = headingTuretaGrade + 360;

            }
            int targetTicks =
                    (int)(headingTuretaGrade * TICKS_PER_DEGREE);
//                            - START_OFFSET_TICKS;
//            int targetTicks = wrapTicks((int)(headingTuretaGrade * TICKS_PER_DEGREE));
//            int currentTicks = wrapTicks(turret.getCurrentPosition());
//            int targetTicks = (int) (headingTuretaGrade * TICKS_PER_DEGREE);
            int currentTicks = turret.getCurrentPosition();
            int error = targetTicks - currentTicks;
            if (currentTicks >= MAX_TICKS && power > 0) power = 0;
            if (currentTicks <= MIN_TICKS && power < 0) power = 0;
            double power =
                    kP * error +
                            kD * (error - lastError);

            lastError = error;

//            double power = pidf(targetTicks, currentTicks);
            power = Math.max(-1, Math.min(1, power));

            turret.setPower(power);
            follower.update();
            telemetry.addData("Current ticks", currentTicks);
            telemetry.addData("Target ticks", targetTicks);
            telemetry.addData("RobotHeading", Math.toDegrees(follower.getHeading()) );
            telemetry.addData("TuretaHeading", headingTuretaGrade);
            telemetry.addData("Alpha", alpha);
            telemetry.addData("Alpha1", alpha1);

            telemetry.addData("Power", power);

            telemetry.update();
        }
    }
    private double pidf(int target, int current) {

        int error = target - current;

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return (kP * error)
                + (kI * integral)
                + (kD * derivative)
                + (kF * Math.signum(error));
    }
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
}

