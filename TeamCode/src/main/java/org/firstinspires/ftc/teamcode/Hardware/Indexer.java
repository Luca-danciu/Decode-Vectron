package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Indexer {
    public Servo IndexerL;
    public Servo IndexerR;

    public Servo Banana;
    public DcMotor Intake;
    public DcMotor ridicatoruDeBilioasa;



    public void indexerinit(HardwareMap hardwareMap){
        IndexerL = hardwareMap.get(Servo.class, "IndexerL");
        IndexerR = hardwareMap.get(Servo.class, "IndexerR");
        Banana = hardwareMap.get(Servo.class, "Banana");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        ridicatoruDeBilioasa = hardwareMap.get(DcMotor.class, "Totr");

        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void KeepInside(){
        Intake.setPower(INTAKE_POWER_KEEP_INSIDE);
    }
    public void Colect(){
        Intake.setPower(INTAKE_POWER_COLLECT);
    }
    public void TakeOut(){
        Intake.setPower(INTAKE_POWER_TAKE_OUT);
    }
    public void TakeOutBit(){
        Intake.setPower(INTAKE_POWER_TAKE_OUT_BIT);
    }
    public void Stop(){
        Intake.setPower(0);
    }

    public void PickPose1(){
        IndexerL.setPosition(INDEXER_PICK_POSE_1);
        IndexerR.setPosition(INDEXER_PICK_POSE_1);
    }
    public void PickPose2(){
        IndexerL.setPosition(INDEXER_PICK_POSE_2);
        IndexerR.setPosition(INDEXER_PICK_POSE_2);
    }
    public void PickPose3(){
        IndexerL.setPosition(INDEXER_PICK_POSE_3);
        IndexerR.setPosition(INDEXER_PICK_POSE_3);
    }
    public void OuttakePose1(){
        IndexerL.setPosition(INDEXER_OUTTAKE_POSE_1);
        IndexerR.setPosition(INDEXER_OUTTAKE_POSE_1);
    }
    public void OuttakePose2(){
        IndexerL.setPosition(INDEXER_OUTTAKE_POSE_2);
        IndexerR.setPosition(INDEXER_OUTTAKE_POSE_2);
    }
    public void OuttakePose3(){
        IndexerL.setPosition(INDEXER_OUTTAKE_POSE_3);
        IndexerR.setPosition(INDEXER_OUTTAKE_POSE_3);
    }
    public double IndexerGetPos(){
        return  IndexerR.getPosition();

    }
    public void Push(){
        Banana.setPosition(INDEXER_PUSH_POSITION);
    }
    public void PushBlue(){
        Banana.setPosition(INDEXER_PUSH_BLUE_POSITION);
    }
    public void Down(){
        Banana.setPosition(INDEXER_DOWN_POSITION);
    }

}
