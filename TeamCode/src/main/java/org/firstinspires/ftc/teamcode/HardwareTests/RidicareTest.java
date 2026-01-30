package org.firstinspires.ftc.teamcode.HardwareTests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Ascent;

@Configurable
@TeleOp
public class RidicareTest extends OpMode {
    Ascent ascent = new Ascent();
    public static double D = 0;
    public static double S = 0;


    @Override
    public void init() {
        ascent.ascentinit(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.cross){
            ascent.Ridicare(D , S);
        }else{
            ascent.Ridicare(0, 0);
        }
        telemetry.addData("D"  ,D );
        telemetry.addData("S"  ,S );

        telemetry.update();

    }
}
