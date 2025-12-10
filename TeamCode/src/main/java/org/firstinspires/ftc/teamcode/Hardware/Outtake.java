package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    public DcMotor Launcher;
    public DcMotor turetaMotor;


    public Servo Angle;
    public void outtakeinit(HardwareMap hardwareMap){

        Launcher = hardwareMap.get(DcMotor.class, "Launcher");
        Angle = hardwareMap.get(Servo.class, "Angle");
        turetaMotor = hardwareMap.get(DcMotor.class, "Tureta");

        Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turetaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turetaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turetaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turetaMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void TuretaAngle(int ticks , double power){
        turetaMotor.setTargetPosition(ticks);
        turetaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turetaMotor.setPower(power);

    }
    public void Charge(){
        Launcher.setPower(-0.8);
    }
    public void StopLauncher(){
        Launcher.setPower(0);
    }

    public void AngleMax(){
        Angle.setPosition(0.83);
    }
    public void AngleMin(){
        Angle.setPosition(0.27);
    }



}
