package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.OpmodesConstants.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ascent {

    public CRServo RidicareDJ;
    public CRServo RidicareDS;
    public CRServo RidicareSJ;
    public CRServo RidicareSS;
    public Servo Prindere;

    public void ascentinit(HardwareMap hardwareMap) {
        RidicareDJ = hardwareMap.get(CRServo.class, "RidicareDJ");
        RidicareDS = hardwareMap.get(CRServo.class, "RidicareDS");
        RidicareSJ = hardwareMap.get(CRServo.class, "RidicareSJ");
        RidicareSS = hardwareMap.get(CRServo.class, "RidicareSS");
        Prindere = hardwareMap.get(Servo.class, "Prindere");

    }
    public void Ridicare( double D ,  double S){
        RidicareDJ.setPower(D);
        RidicareDS.setPower(D);
        RidicareSJ.setPower(S);
        RidicareSS.setPower(S);
    }
    public void Prins(){
        Prindere.setPosition(ASCENT_GRIP_POSITION);
    }
    public void Eliberat(){
        Prindere.setPosition(ASCENT_RELEASE_POSITION);
    }
}

