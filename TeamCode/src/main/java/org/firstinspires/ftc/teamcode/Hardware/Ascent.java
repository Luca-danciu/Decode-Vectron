package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ascent {

    CRServo RidicareDJ;
    CRServo RidicareDS;
    CRServo RidicareSJ;
    CRServo RidicareSS;

    public void ascentinit(HardwareMap hardwareMap) {
        RidicareDJ = hardwareMap.get(CRServo.class, "RidicareDJ");
        RidicareDS = hardwareMap.get(CRServo.class, "RidicareDS");
        RidicareSJ = hardwareMap.get(CRServo.class, "RidicareSJ");
        RidicareSS = hardwareMap.get(CRServo.class, "RidicareSS");

    }
    public void Ridicare( double D ,  double S){
        RidicareDJ.setPower(D);
        RidicareDS.setPower(D);
        RidicareSJ.setPower(S);
        RidicareSS.setPower(S);
    }
}

