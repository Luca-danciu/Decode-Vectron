package org.firstinspires.ftc.teamcode.HardwareTests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Indexer;
@Disabled
@TeleOp
@Configurable

public class IndexerPosition extends OpMode {
    public static double p=0;
    Indexer indexer = new Indexer();
    @Override
    public void init() {
        indexer.indexerinit(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.square) indexer.PickPose1();
        if (gamepad1.triangle) indexer.PickPose2();
        if (gamepad1.circle) indexer.PickPose3();

        if (gamepad1.dpad_left) indexer.OuttakePose1();
        if (gamepad1.dpad_up) indexer.OuttakePose2();
        if (gamepad1.dpad_right) indexer.OuttakePose3();

        if (gamepad1.ps){
            indexer.IndexerL.setPosition(p);
            indexer.IndexerR.setPosition(p);
        }
        telemetry.addData("POS:", indexer.IndexerR.getPosition());
        telemetry.addData("POS functie", indexer.IndexerGetPos());
        telemetry.update();
    }
}
