package org.firstinspires.ftc.teamcode.HardwareTests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Ascent;

//@Disabled
@Configurable
@TeleOp
public class RidicareTest extends OpMode {
    Ascent ascent = new Ascent();
    public static double D = 1;
    public static double S = 1;


    @Override
    public void init() {
        ascent.ascentinit(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.square){
            ascent.RidicareSJ.setPower(D);
            ascent.RidicareSS.setPower(D);
        }
        if (gamepad1.circle){
            ascent.RidicareDJ.setPower(D);
            ascent.RidicareDS.setPower(D);
        }
        if (gamepad1.cross){
            ascent.Ridicare(D , S);
        }else{
            ascent.Ridicare(0, 0);
        }
        if (gamepad1.dpad_down){
            ascent.Eliberat();
        }
        if (gamepad1.dpad_up){
            ascent.Prins();
        }
        telemetry.addData("D"  ,D );
        telemetry.addData("S"  ,S );

        telemetry.update();

    }
}
