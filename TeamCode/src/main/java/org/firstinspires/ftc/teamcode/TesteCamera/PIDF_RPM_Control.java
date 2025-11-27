package org.firstinspires.ftc.teamcode.TesteCamera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.PIDController;

@TeleOp(name="PIDF RPM Control")
public class PIDF_RPM_Control extends LinearOpMode {

    DcMotor motor;
    public static double p = 0.0009 , i = 0.001, d = 0.000012, f = 0.00000006;
    public static int target = 0;
    public final double ticks_in_degree = 1.493;

    public Indexer indexer = new Indexer();
    double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class, "Launcher");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController controller = new PIDController(p, i, d);
        indexer.indexerinit(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {

            int liftPos = motor.getCurrentPosition();
            double pid = controller.calculate(liftPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            motor.setPower(power);
            if (gamepad1.cross){
                target += 500;
                power = pid + ff;

            }
            if(gamepad1.circle){
                target = -motor.getCurrentPosition();
                power = 0;
            }
            if (gamepad1.dpad_up){
                indexer.Push();
            }
if (gamepad1.dpad_down){
    indexer.Down();
}
            telemetry.addData("target", target);
            telemetry.addData("Current", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
