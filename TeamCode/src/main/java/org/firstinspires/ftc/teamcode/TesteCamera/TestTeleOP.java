package org.firstinspires.ftc.teamcode.TesteCamera;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
import org.firstinspires.ftc.teamcode.Hardware.IMUIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.WebcamTureta;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class TestTeleOP extends LinearOpMode {

    public DcMotor FrontLeft, FrontRight, RearLeft, RearRight;
    public Indexer indexer = new Indexer();
    public Outtake outtake = new Outtake();
    public WebcamTureta webcamTureta = new WebcamTureta();
    public DistanceSensor distanceSensor;
    public IMUIndexer imuIndexer = new IMUIndexer();

    ColorSensorIndexer colorSensorIndexer = new ColorSensorIndexer();

    double heading = 0;
    double initialHeading = 0;

    DcMotor turetaMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        // inițializări
        RearLeft = hardwareMap.get(DcMotor.class, "RL");
        RearRight = hardwareMap.get(DcMotor.class, "RR");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        turetaMotor = hardwareMap.get(DcMotor.class, "Tureta");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");

        colorSensorIndexer.initcolorsensor(hardwareMap);
        indexer.indexerinit(hardwareMap);
        outtake.outtakeinit(hardwareMap);
        webcamTureta.initwebcam(hardwareMap);
        imuIndexer.init(hardwareMap);

        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        turetaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turetaMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turetaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turetaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        initialHeading = imuIndexer.getHeading(AngleUnit.DEGREES);

        double TICKS_PER_REV = 537.7;
        double GEAR_RATIO = 3.67;
        double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
        boolean urmarireActivata = false;
        double turetaHeadingFix = 0;

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.dpad_up) indexer.OuttakePose2();
            if (gamepad1.dpad_left) indexer.OuttakePose1();
            if (gamepad1.dpad_right) indexer.OuttakePose3();

        }
    }
}
