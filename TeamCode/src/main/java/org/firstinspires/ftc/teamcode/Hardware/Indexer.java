package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Indexer {
    public Servo IndexerL;
    public Servo IndexerR;

    public Servo Banana;
    public DcMotor Intake;


    public void indexerinit(HardwareMap hardwareMap){
        IndexerL = hardwareMap.get(Servo.class, "IndexerL");
        IndexerR = hardwareMap.get(Servo.class, "IndexerR");
        Banana = hardwareMap.get(Servo.class, "Banana");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void KeepInside(){
        Intake.setPower(-0.7);
    }
    public void Colect(){
        Intake.setPower(-1);

    }
    public void TakeOut(){
        Intake.setPower(1);
    }
    public void Stop(){
        Intake.setPower(0);
    }

    public void PickPose1(){
        IndexerL.setPosition(0.025);
        IndexerR.setPosition(0.025);
    }
    public void PickPose2(){
        IndexerL.setPosition(0.395);
        IndexerR.setPosition(0.395);
    }
    public void PickPose3(){
        IndexerL.setPosition(0.76);
        IndexerR.setPosition(0.76);
    }
    public void OuttakePose1(){
        IndexerL.setPosition(0.21);
        IndexerR.setPosition(0.21);
    }
    public void OuttakePose2(){
        IndexerL.setPosition(0.58);
        IndexerR.setPosition(0.58);
    }
    public void OuttakePose3(){
        IndexerL.setPosition(0.96);
        IndexerR.setPosition(0.96);
    }
    public double IndexerGetPos(){
        return  IndexerR.getPosition();

    }
    public void Push(){
        Banana.setPosition(0.3);
    }
    public void PushBlue(){
        Banana.setPosition(0.36);
    }
    public void Down(){
        Banana.setPosition(0.7);
    }

}
