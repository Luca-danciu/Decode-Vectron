package org.firstinspires.ftc.teamcode.HardwareTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Indexer;
//@Disabled
@TeleOp
public class ColorSensorTest extends OpMode {
    ColorSensorIndexer bench = new ColorSensorIndexer();
    ColorSensorIndexer.DetectedColor detectedColor ;
    public DistanceSensor distanceSensor;

    Indexer indexer = new Indexer();

    @Override
    public void init() {
        bench.initcolorsensor(hardwareMap);
        indexer.indexerinit(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");


    }

    @Override
    public void loop() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
//71
        if(gamepad1.cross) indexer.KeepInside();
        else indexer.Stop();
        if (gamepad1.square) indexer.PickPose1(); // 69
        if (gamepad1.triangle) indexer.PickPose2(); //67
        if (gamepad1.circle) indexer.PickPose3(); //67
        detectedColor = bench.getDetectedColor();
        telemetry.addData("Detected color ", detectedColor);
        telemetry.addData("Dist ", distance);
        bench.sendRGBTelemetry(telemetry);

        telemetry.update();
    }
}
