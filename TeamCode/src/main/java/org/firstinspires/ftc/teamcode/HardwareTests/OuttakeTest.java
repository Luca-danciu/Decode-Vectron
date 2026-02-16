package org.firstinspires.ftc.teamcode.HardwareTests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Indexer;

@Configurable
@TeleOp(name="Launcher")
public class OuttakeTest extends LinearOpMode {

    DcMotorEx flywheel;
    DcMotor totr;

//    public static double P = 30;
    public static double P = 100;
    public static double I = 0;
    public static double D = 0;
    public static double F = 100;
    Indexer indexer = new Indexer();
    Servo angle;
    boolean next1 = false;
    boolean next2 = false;

    static final double MAX_TICKS_PER_SEC = 1800;
    double targetVelocity = 0;
    double s = 0;

    @Override
    public void runOpMode() {
        ElapsedTime dpad = new ElapsedTime();
        flywheel = hardwareMap.get(DcMotorEx.class, "Launcher" );
        totr = hardwareMap.get(DcMotor.class, "Totr" );
        angle  = hardwareMap.get(Servo.class , "Angle");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        totr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        indexer.indexerinit(hardwareMap);


        // PIDF pentru 6000 RPM flywheel (bun punct de start)
//        flywheel.setVelocityPIDFCoefficients(
//                30,   // P
//                0,    // I
//                2,    // D
//                12    // F (feedforward foarte important la flywheel)
//        );
        waitForStart();

        while (opModeIsActive()) {
            flywheel.setVelocityPIDFCoefficients(P, I, D, F);

            if (gamepad1.left_bumper) {
                targetVelocity = MAX_TICKS_PER_SEC;
            }
            if (gamepad1.dpad_left) {
                targetVelocity -= 50;
            }

            if (gamepad1.dpad_right) {
                targetVelocity += 50;
            }
            if (gamepad1.options){
                targetVelocity = 0;
            }
            if (s < 0){
                s = 0;
            }
            if ( s  > 0.7){
                s = 0.7;
            }
            if (gamepad1.dpad_up ){
                s += 0.02;
                dpad.reset();
            }
            if (gamepad1.dpad_down){
                s -= 0.02;
                dpad.reset();
            }
            angle.setPosition(s);

            if (gamepad1.square){
                indexer.OuttakePose1();
                indexer.KeepInside();
                next1 = true;
                s = 0.25;
                dpad.reset();
            }
//            && (flywheel.getVelocity() < targetVelocity + 50 ) && (flywheel.getVelocity() > targetVelocity - 60 )
            if (next1  &&  dpad.milliseconds() > 400){
                //900
                indexer.OuttakePose2();
                next1 = false;
                next2 = true;
                s = 0.5;
                dpad.reset();
            }
            if (next2 && dpad.milliseconds() > 400){
                //1200
                indexer.OuttakePose3();
                next2 = false;
                indexer.Stop();
            }
//            if (next2 && (flywheel.getVelocity() < targetVelocity + 50 ) && (flywheel.getVelocity() > targetVelocity - 60 ) && dpad.milliseconds() > 300){
//                indexer.OuttakePose3();
//                indexer.Stop();
//                next2 = false;
//            }


            if (gamepad1.right_bumper){
                totr.setPower(1);
            }
            if (gamepad1.ps){
                totr.setPower(0);
            }
            if (gamepad1.triangle){
                indexer.PickPose1();
            }
            if (gamepad1.circle){
                indexer.PickPose2();
            }
            if (gamepad1.cross){
                indexer.PickPose3();
            }

            if (gamepad1.right_trigger > 0.5 ){
                indexer.KeepInside();
            }

            flywheel.setVelocity(targetVelocity);


            telemetry.addData("Servo" , s);
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
