package org.firstinspires.ftc.teamcode.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Indexer;
@Disabled
@TeleOp
public class ColorSensorTest extends OpMode {
    ColorSensorIndexer bench = new ColorSensorIndexer();
    ColorSensorIndexer.DetectedColor detectedColor ;
    Indexer indexer = new Indexer();

    @Override
    public void init() {
        bench.initcolorsensor(hardwareMap);
        indexer.indexerinit(hardwareMap);

    }

    @Override
    public void loop() {
        if(gamepad1.cross) indexer.KeepInside();
        else indexer.Stop();
        if (gamepad1.square) indexer.PickPose1();
        if (gamepad1.triangle) indexer.PickPose2();
        if (gamepad1.circle) indexer.PickPose3();
        detectedColor = bench.getDetectedColor();
        telemetry.addData("Detected color ", detectedColor);
        telemetry.update();
    }
}
