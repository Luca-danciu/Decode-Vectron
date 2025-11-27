package org.firstinspires.ftc.teamcode.TesteCamera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;

@TeleOp
public class ColorSensorTest extends OpMode {
    ColorSensorIndexer bench = new ColorSensorIndexer();
    ColorSensorIndexer.DetectedColor detectedColor ;
    @Override
    public void init() {
        bench.initcolorsensor(hardwareMap);
    }

    @Override
    public void loop() {

        detectedColor = bench.getDetectedColor();
        telemetry.addData("Detected color ", detectedColor);
        telemetry.update();
    }
}
