package org.firstinspires.ftc.teamcode.HardwareTests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.IMUIndexer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled

@TeleOp
public class TuretaTeleOp extends OpMode {
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor RearLeft;
    public DcMotor RearRight;

    private Follower follower;
    public int towerX = 70;
    public int towerY = -70;
    IMUIndexer imuIndexer = new IMUIndexer();
    private DcMotorEx turret;
    static final double TICKS_PER_DEGREE = 6.195;
    double robotX = 0;
    double robotY = 0;
    double dx = 0;
    double dy = 0;
    double headingRobot = 0;
    double alpha1 = 0;
    double headingTuretaGrade;
    double targetTicks = 0;
    double power;
//    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREE);

    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0002;
    public static double kF = 0.05;

    // ================= PID =================
    private double integral = 0;
    private double lastError = 0;
    int stopTracking = 0;

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
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = Constants.createFollower(hardwareMap);
        imuIndexer.init(hardwareMap);
    }

    // definește limitele turetei
    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREE);
    public static final int MIN_TICKS = -MAX_TICKS;

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

        switch (stopTracking){
            case 0:
                robotX = follower.getPose().getX();
                robotY = follower.getPose().getY();
                headingRobot = imuIndexer.getHeading(AngleUnit.DEGREES);

                dx = towerX - robotX;
                dy = towerY - robotY;

                alpha1 = Math.toDegrees(Math.atan2(dy, dx)); // unghiul spre țintă
                headingTuretaGrade = alpha1 - headingRobot; // eroarea unghiului

                // normalizează între -180 și 180
                headingTuretaGrade = ((headingTuretaGrade + 180) % 360) - 180;

                targetTicks = headingTuretaGrade * TICKS_PER_DEGREE;
                targetTicks = wrapTicks((int) targetTicks);

                // limitare fizică: nu depăși capetele mecanice
                if (targetTicks > MAX_TICKS) targetTicks = MAX_TICKS;
                if (targetTicks < MIN_TICKS) targetTicks = MIN_TICKS;

                int currentTicks = wrapTicks(turret.getCurrentPosition());

                // calculează puterea folosind PID
                power = pidf((int) targetTicks, currentTicks);

                // limitează puterea între -1 și 1
                power = Math.max(-1, Math.min(1, power));

                // oprește motorul dacă suntem deja la capăt
                if ((currentTicks >= MAX_TICKS && power > 0) || (currentTicks <= MIN_TICKS && power < 0)) {
                    power = 0;
                }

                turret.setPower(power);

                follower.update();
                telemetry.addData("Current ticks", currentTicks);
                if (gamepad1.options){
                    stopTracking = 1;
                }
                break;
            case 1:
                turret.setPower(0);
                if (gamepad1.options){
                    stopTracking = 0;
                }
                break;
        }

        telemetry.addData("state" , stopTracking );
        telemetry.addData("Target ticks", targetTicks);
        telemetry.addData("Power", power);
    }


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

}
