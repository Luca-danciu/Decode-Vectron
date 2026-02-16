package org.firstinspires.ftc.teamcode.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Indexer;

@TeleOp(name="Launcher")
public class OuttakeTest extends LinearOpMode {

    DcMotorEx flywheel;
    Indexer indexer = new Indexer();
    Servo angle;

    static final double MAX_TICKS_PER_SEC = 2800;
    double target = 0.3;
    double s = 0.5;

    @Override
    public void runOpMode() {
        ElapsedTime dpad = new ElapsedTime();
        flywheel = hardwareMap.get(DcMotorEx.class, "Launcher" );
        angle  = hardwareMap.get(Servo.class , "Angle");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.indexerinit(hardwareMap);


        // PIDF pentru 6000 RPM flywheel (bun punct de start)
//        flywheel.setVelocityPIDFCoefficients(
//                30,   // P
//                0,    // I
//                2,    // D
//                12    // F (feedforward foarte important la flywheel)
//        );
        dpad.reset();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left){
                indexer.Push();
            }
            if (gamepad1.dpad_right){
                indexer.Down();
            }

//            if (gamepad1.left_bumper) {
//                target = MAX_TICKS_PER_SEC;
//            }
//
//            else if (gamepad1.right_trigger > 0.05) {
//                target = MAX_TICKS_PER_SEC / 2;
//            }
//
//            else if (gamepad1.x) {
//                target = 0;
//            }
//            flywheel.setVelocity(target);
            if (gamepad1.dpad_up && dpad.milliseconds() > 500){
                s += 0.02;
                dpad.reset();
            }
            if (gamepad1.dpad_down && dpad.milliseconds() > 500){
                s -= 0.02;
                dpad.reset();
            }
            angle.setPosition(s);
            if (gamepad1.triangle){
                flywheel.setPower(1);
            }
            if (gamepad1.cross){
                flywheel.setPower(0);
            }

            telemetry.addData("Servo" , s);
            telemetry.addData("Target", target);
            telemetry.addData("Actual", flywheel.getVelocity());
            telemetry.update();
        }
    }
}
