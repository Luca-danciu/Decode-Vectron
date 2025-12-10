package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    Outtake outtake = new Outtake();
    static final double TICKS_PER_DEGREE = 6.195;
    double robotX = 0;
    double robotY = 0;
    double dx = 0;
    double dy = 0;
    double headingRobot = 0;
    double alpha1 = 0;
    double headingTuretaGrade;
    double targetTicks;
    double power;

    @Override
    public void init() {
        outtake.outtakeinit(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        imuIndexer.init(hardwareMap);
    }

    @Override
    public void loop() {
        robotX = follower.getPose().getX();
        robotY = follower.getPose().getY();
        headingRobot = imuIndexer.getHeading(AngleUnit.DEGREES);
//        19734125
        dx = robotX + 40;
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
        outtake.TuretaAngle((int) targetTicks , (double) power);

        follower.update();
//11.245
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Heading Robot", headingRobot);
        telemetry.addData("Heading tureta (cat crede ea ca a mers )" , (( 180 - alpha1 ) - headingRobot) );
        telemetry.addData("Heading tureta" , headingTuretaGrade);


    }
}
