package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.IMUIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class TuretaTeleOp extends OpMode {
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
    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREE);

    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0002;
    public static double kF = 0.05;

    // ================= PID =================
    private double integral = 0;
    private double lastError = 0;
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "Tureta");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = Constants.createFollower(hardwareMap);
        imuIndexer.init(hardwareMap);
    }

    @Override
    public void loop() {
        robotX = follower.getPose().getX();
        robotY = follower.getPose().getY();
        headingRobot = imuIndexer.getHeading(AngleUnit.DEGREES);
//        19734125
        dx = robotX + 10;
        dy = 140 - robotY;


        alpha1 = Math.toDegrees(Math.atan2(dy , dx));
        headingTuretaGrade = (( 180 - alpha1 ) - headingRobot);
        targetTicks = headingTuretaGrade * TICKS_PER_DEGREE;
        if (targetTicks < 0){
            power = -1 ;
            targetTicks = targetTicks * (-1);
        }else if (targetTicks > 0){
            power = 1;
        }else {
            power = 0;
        }
        double targetPosition = targetTicks;

        targetPosition = wrapTicks((int) targetPosition);

        int currentPosition = wrapTicks(turret.getCurrentPosition());


        turret.setPower(power);

//        outtake.TuretaAngle((int) targetTicks , (double) power);

        follower.update();
//11.245
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Heading Robot", headingRobot);
        telemetry.addData("Heading tureta (cat crede ea ca a mers )" , (( 180 - alpha1 ) - headingRobot) );
        telemetry.addData("Heading tureta" , headingTuretaGrade);


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
