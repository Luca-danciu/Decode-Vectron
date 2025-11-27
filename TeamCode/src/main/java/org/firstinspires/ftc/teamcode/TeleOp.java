package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
import org.firstinspires.ftc.teamcode.Hardware.IMUIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.WebcamTureta;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")

public class TeleOp extends LinearOpMode {

//Motoare/Componente
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor RearLeft;
    public DcMotor RearRight;
    public DistanceSensor distanceSensor;
    public DcMotor launcher;


    //Hardware
    public Indexer indexer = new Indexer();
    public Outtake outtake = new Outtake();
    public WebcamTureta webcamTureta = new WebcamTureta();
    public IMUIndexer imuIndexer = new IMUIndexer();
    public ColorSensorIndexer colorSensorIndexer = new ColorSensorIndexer();

//Valori
    double heading = 0;
    double initialHeading = 0;
    public double distance;
    int purplecount = 0;
    int greencount = 0;
    double TICKS_PER_REV = 537.7;
    double GEAR_RATIO = 5.4;
    double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
    double turetaHeadingFix = 0;

//Verificari
    boolean ballsRemoved = false;
    boolean detectionActive = true;
    boolean allBallsIn  = false;
    boolean newCaseDetected = false;
    boolean urmarireActivata = false;
    boolean colectareSelectiva = false;
    boolean stopIntake = false;

//Stringuri
    String detectedCase = "NONE";
    public String pp1 = "PickPose1";
    public String pp2 = "PickPose2";
    public String pp3 = "PickPose3";
    String greenBallPickedAt = "Nicio bila verde preluata";
    ColorSensorIndexer.DetectedColor detectedColor1, detectedColor2, detectedColor3;

//PIDF
    public static double p = 0.01, i = 0.001, d = 0.000012, f = 0.00000006;
    public static int target = 5000;
    public final double ticks_in_degree = 1.493;
    double power = 0;

    public void runOpMode() throws InterruptedException {

        int stateThrow = 0;
        int stateCollect = 0;

        ElapsedTime throwTimer = new ElapsedTime();
        ElapsedTime collecttimer = new ElapsedTime();
        ElapsedTime TakeGreenBallOut = new ElapsedTime();
        ElapsedTime TakePurpleBallOut = new ElapsedTime();

        RearLeft = hardwareMap.get(DcMotor.class, "RL");
        RearRight = hardwareMap.get(DcMotor.class, "RR");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");
        launcher = hardwareMap.get(DcMotor.class, "Launcher");
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PIDController controller = new PIDController(p, i, d);
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


        waitForStart();

        initialHeading = imuIndexer.getHeading(AngleUnit.DEGREES);
        indexer.PickPose1();
        double pid = 0;
        double ff = 0;

        while (opModeIsActive() && !isStopRequested()) {

            distance = distanceSensor.getDistance(DistanceUnit.MM);

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)));
            FrontLeft.setPower((forward + strafe + turn) / denominator * 2);
            RearLeft.setPower((forward - (strafe - turn)) / denominator * 2);
            FrontRight.setPower((forward - (strafe + turn)) / denominator * 2);
            RearRight.setPower((forward + (strafe - turn)) / denominator * 2);

            String tag = webcamTureta.detectTag();
            double dist = webcamTureta.getDistanceMeters();

            int liftPos = launcher.getCurrentPosition();
            double pid0 = controller.calculate(liftPos, target);
            double ff0 = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            launcher.setPower(power);

            if (detectionActive) {
                tag = webcamTureta.detectTag();
                if (tag != null && (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {
                    detectedCase = tag;
                    gamepad1.rumble(1000);
                    newCaseDetected = true;
                }
            }

            if (dist > 1.70 && dist < 2.5) {outtake.AngleMax();
            } else if (dist < 1.70 && dist > 1.50) {outtake.Angle.setPosition(0.65);
            } else if (dist < 1.50 && dist > 1.00) {outtake.Angle.setPosition(0.5);
            } else if (dist < 1){outtake.AngleMin();}


//power / dist
            if (dist < 1.25){
                pid = -0.7;
                ff = 0;
            }else if (dist > 1.25 && dist < 1.50){
                pid = -0.75;
                ff = 0;
            }else if (dist > 1.50 && dist < 1.70){
                pid = -0.8;
                ff = 0;
            }else if (dist >1.70 && dist < 1.85){
                pid = -0.8;
                ff = 0;
            } else if (dist > 1.85 && dist < 2.5) {
                pid = -0.9;
                ff = 0;
            }else if (dist > 2.5){
                outtake.Angle.setPosition(0.73);
                pid = pid0;
                ff = ff0;
            }


//Porneste detectarea
            if (gamepad1.right_bumper) {
                detectionActive = true;
                webcamTureta.startDetection();
            }
//Opreste detectarea
            else {
                detectionActive = false;
                webcamTureta.stopDetection();
            }


//Aruncare
            switch (stateThrow) {
                // ----------------- PRIMA BILA -----------------
                case 0:
                    if (gamepad1.triangle && allBallsIn) {
                        stopIntake = false;
                        power = pid + ff;
                        if(!colectareSelectiva){
                            throwTimer.reset();
                            stateThrow = 1;
                        } else{
                            if (greenBallPickedAt.equals(pp1)) {
                                String caseUsed = detectedCase;
                                switch (caseUsed) {
                                    case "GPP":
                                        indexer.OuttakePose2();
                                        break;
                                    case "PGP":
                                        indexer.OuttakePose3();
                                        break;
                                    case "PPG":
                                        indexer.OuttakePose3();
                                        break;
                                    default:
                                        indexer.OuttakePose1();
                                        break;
                                }
                            }
                            if (greenBallPickedAt.equals(pp2)) {
                                String caseUsed = detectedCase;

                                switch (caseUsed) {
                                    case "GPP":
                                        indexer.OuttakePose3();
                                        break;
                                    case "PGP":
                                        indexer.OuttakePose2();
                                        break;
                                    case "PPG":
                                        indexer.OuttakePose2();
                                        break;
                                    default:
                                        indexer.OuttakePose1();
                                        break;
                                }
                            }
                            if (greenBallPickedAt.equals(pp3)) {
                                String caseUsed = detectedCase;

                                switch (caseUsed) {
                                    case "GPP":
                                        indexer.OuttakePose1();
                                        break;
                                    case "PGP":
                                        indexer.OuttakePose2();
                                        break;
                                    case "PPG":
                                        indexer.OuttakePose2();
                                        break;
                                    default:
                                        indexer.OuttakePose1();
                                        break;
                                }
                            }
                            throwTimer.reset();
                            stateThrow = 1;
                        }
                    }
                    break;

                case 1:
                    if (throwTimer.milliseconds() > 1300) {
                        indexer.Push();
                        throwTimer.reset();
                        stateThrow = 2;
                    }
                    break;

                case 2:
                    if (throwTimer.milliseconds() > 500) {
                        indexer.Down();
                        power = pid + ff;
                        throwTimer.reset();
                        stateThrow = 3;
                    }
                    break;

                // ----------------- A DOUA BILA -----------------
                case 3:
                    if (throwTimer.milliseconds() > 300) {
                        if (!colectareSelectiva){
                            indexer.OuttakePose2();
                            throwTimer.reset();
                            stateThrow = 4;
                        }else {
                            if (greenBallPickedAt.equals(pp1)) {
                                String caseUsed = detectedCase;

                                switch (caseUsed) {
                                    case "GPP":
                                        indexer.OuttakePose3();
                                        break;
                                    case "PGP":
                                        indexer.OuttakePose2();
                                        break;
                                    case "PPG":
                                        indexer.OuttakePose1();
                                        break;
                                    default:
                                        indexer.OuttakePose2();
                                        break;
                                }
                            }
                            if (greenBallPickedAt.equals(pp2)) {
                                String caseUsed = detectedCase;

                                switch (caseUsed) {
                                    case "GPP":
                                        indexer.OuttakePose2();
                                        break;
                                    case "PGP":
                                        indexer.OuttakePose3();
                                        break;
                                    case "PPG":
                                        indexer.OuttakePose1();
                                        break;
                                    default:
                                        indexer.OuttakePose2();
                                        break;
                                }
                            }
                            if (greenBallPickedAt.equals(pp3)) {
                                String caseUsed = detectedCase;

                                switch (caseUsed) {
                                    case "GPP":
                                        indexer.OuttakePose2();
                                        break;
                                    case "PGP":
                                        indexer.OuttakePose1();
                                        break;
                                    case "PPG":
                                        indexer.OuttakePose3();
                                        break;
                                    default:
                                        indexer.OuttakePose2();
                                        break;
                                }
                            }
                            throwTimer.reset();
                            stateThrow = 4;
                        }


                    }
                    break;

                case 4:
                    if (throwTimer.milliseconds() > 700) {
                        indexer.Push();
                        throwTimer.reset();
                        stateThrow = 5;
                    }
                    break;

                case 5:
                    if (throwTimer.milliseconds() > 300) {
                        indexer.Down();
                        power = pid + ff;
                        throwTimer.reset();
                        stateThrow = 6;
                    }
                    break;

                // ----------------- A TREIA BILA -----------------
                case 6:
                    if (throwTimer.milliseconds() > 600) {
                        if (!colectareSelectiva){
                            indexer.OuttakePose3();
                            throwTimer.reset();
                            stateThrow = 7;
                        }else{
                        if (greenBallPickedAt.equals(pp1)) {
                            String caseUsed = detectedCase;

                            switch (caseUsed) {
                                case "GPP":
                                    indexer.OuttakePose1();
                                    break;
                                case "PGP":
                                    indexer.OuttakePose1();
                                    break;
                                case "PPG":
                                    indexer.OuttakePose2();
                                    break;
                                default:
                                    indexer.OuttakePose3();
                                    break;
                            }
                        }
                        if (greenBallPickedAt.equals(pp2)) {
                            String caseUsed = detectedCase;

                            switch (caseUsed) {
                                case "GPP":
                                    indexer.OuttakePose1();
                                    break;
                                case "PGP":
                                    indexer.OuttakePose1();
                                    break;
                                case "PPG":
                                    indexer.OuttakePose3();
                                    break;
                                default:
                                    indexer.OuttakePose3();
                                    break;
                            }
                        }
                        if (greenBallPickedAt.equals(pp3)) {
                            String caseUsed = detectedCase;

                            switch (caseUsed) {
                                case "GPP":
                                    indexer.OuttakePose3();
                                    break;
                                case "PGP":
                                    indexer.OuttakePose3();
                                    break;
                                case "PPG":
                                    indexer.OuttakePose1();
                                    break;
                                default:
                                    indexer.OuttakePose3();
                                    break;
                            }
                        }
                            throwTimer.reset();
                            stateThrow = 7;
                        }

                    }
                    break;

                case 7:
                    if (throwTimer.milliseconds() > 300) {
                        indexer.Push(); // aruncă a treia bilă
                        throwTimer.reset();
                        stateThrow = 8;
                    }
                    break;

                case 8:
                    if (throwTimer.milliseconds() > 500) {
                        if (colectareSelectiva){
                            stateCollect = 96;
                        }else{
                            stateCollect = 0;
                        }
                        power = 0;
                        indexer.Down();
                        purplecount = 0;
                        greencount = 0;
                        stopIntake = false;
                        allBallsIn = false;
                        stateThrow = 0;
                        greenBallPickedAt = "Nicio bila verde preluata";
                    }
                    break;
            }

//Colectare
            switch (stateCollect) {
                case 0: // PickPose1
                    indexer.PickPose1();
                    if (distance <= 60) {
                        indexer.PickPose2();
                        collecttimer.reset();
                        ballsRemoved = false;
                        stateCollect = 1;

                    }

            break;

            case 1:
                if (distance <= 60 && collecttimer.milliseconds() > 600) {
                    indexer.PickPose3();
                    collecttimer.reset();
                    stateCollect = 2;
                }
                break;

            case 2: // PickPose3
                if (distance <= 60 && collecttimer.milliseconds() > 300) {
                    collecttimer.reset();
                    indexer.OuttakePose1();
                    allBallsIn = true;
                    stopIntake = true;
                    stateCollect = 3;
                }

                break;

            case 3:
                if (ballsRemoved) {
                    collecttimer.reset();
                    stopIntake = false;
                    allBallsIn = false;
                    stateCollect = 0;
                }
                break;

            case 96:
                indexer.KeepInside();
                indexer.PickPose1();
                detectedColor1 = colorSensorIndexer.getDetectedColor();
                if (distance <= 60) {
                    if (detectedColor1 == ColorSensorIndexer.DetectedColor.PURPLE) {
                        purplecount++;
                        indexer.PickPose2();
                        collecttimer.reset();
                        ballsRemoved = false;
                        detectedColor1 = ColorSensorIndexer.DetectedColor.UNKNOWN;
                        stateCollect = 97;
                    } else if (detectedColor1 == ColorSensorIndexer.DetectedColor.GREEN) {
                        greencount++;
                        if (greencount == 1) {
                            greenBallPickedAt = "PickPose1";
                        }
                        indexer.PickPose2();
                        collecttimer.reset();
                        ballsRemoved = false;
                        stateCollect = 97;
                    }
                }
            break;

            case 97: // PickPose2
                detectedColor2 = colorSensorIndexer.getDetectedColor();
                if (distance <= 60 && collecttimer.milliseconds() > 600) {
                    if (detectedColor2 == ColorSensorIndexer.DetectedColor.PURPLE) {
                        purplecount++;
                        collecttimer.reset();
                        indexer.PickPose3();
                        stateCollect = 98;
                    } else if (detectedColor2 == ColorSensorIndexer.DetectedColor.GREEN) {
                        greencount++;
                        if (greencount == 1) {
                            greenBallPickedAt = "PickPose2";
                        }
                        collecttimer.reset();
                        if (greencount == 1) {
                            indexer.PickPose3();
                            stateCollect = 98;
                        } else if (greencount > 1) {
                            TakeGreenBallOut.reset();
                        }
                    }
                }
            break;

            case 98: // PickPose3
                detectedColor3 = colorSensorIndexer.getDetectedColor();
                if (distance <= 60 && collecttimer.milliseconds() > 300) {
                    if (detectedColor3 == ColorSensorIndexer.DetectedColor.PURPLE) {
                        purplecount++;
                        collecttimer.reset();
                        if (purplecount > 2) {
                            TakePurpleBallOut.reset();
                        }
                    } else if (detectedColor3 == ColorSensorIndexer.DetectedColor.GREEN) {
                        greencount++;
                        if (greencount == 1) {
                            greenBallPickedAt = "PickPose3";
                        }
                        collecttimer.reset();
                        if (greencount > 1) {
                            TakeGreenBallOut.reset();
                        }
                    }
                    if (greencount == 1 && purplecount == 2) {
                        indexer.OuttakePose1();
                        stopIntake = true;
                        allBallsIn = true;
                        stateCollect = 99;
                    }
                }
            break;

            case 99:
                if (ballsRemoved) {
                    collecttimer.reset();
                    allBallsIn = false;
                    stateCollect = 96;
                }
            break;}


//Resetare colectarea selectiva
            if (gamepad1.ps) {
                stateCollect = 96;
                colectareSelectiva = true;
                stopIntake = false;
                greencount = 0;
                purplecount = 0;
                greenBallPickedAt = "Nicio bila verde preluata";
            }

//Intake Role
            if (gamepad1.left_trigger > 0) {
                indexer.Colect();
            } else if (gamepad1.right_trigger > 0) {
                indexer.TakeOut();
            } else if (TakeGreenBallOut.milliseconds() < 400) {
                indexer.TakeOut();
                greencount = 1;
            } else if (TakePurpleBallOut.milliseconds() < 400) {
                indexer.TakeOut();
                purplecount = 2;
            }else if(stopIntake && collecttimer.milliseconds() > 700){
                indexer.Stop();
            }else {
                indexer.KeepInside();
            }

//Unghi Tureta
            if (gamepad1.cross) outtake.AngleMin();
            if (gamepad1.circle) outtake.AngleMax();

            if(gamepad1.options) {
                stateCollect = 0;
                stopIntake = false;
            }

            heading = imuIndexer.getHeading(AngleUnit.DEGREES);
            double headingDifference = heading - initialHeading;

            if (gamepad1.dpad_left) {
                webcamTureta.startDetection();
                String tagName = webcamTureta.detectTag();

                if (tagName.equals("Blue Tower")) {
                    urmarireActivata = true;
                    turetaHeadingFix = imuIndexer.getHeading(AngleUnit.DEGREES);
                }
            }

            if (gamepad1.dpad_right) {
                urmarireActivata = false;
                webcamTureta.stopDetection();
                outtake.turetaMotor.setTargetPosition(0);
                outtake.turetaMotor.setPower(1);
            }


            if (urmarireActivata) {
                double diferentaHeading = imuIndexer.getHeading(AngleUnit.DEGREES) - turetaHeadingFix;

                int targetPosition = (int)(diferentaHeading * TICKS_PER_DEGREE);
                outtake.turetaMotor.setTargetPosition(targetPosition);
                outtake.turetaMotor.setPower(1);
            } else {
                outtake.turetaMotor.setTargetPosition(0);
            }

            telemetry.addData("Stare camera", webcamTureta.isDetecting() ? "Pornita" : "Oprita");
            telemetry.addData("Distance" , dist);
            telemetry.addData("Bile mov", purplecount);
            telemetry.addData("Bile verzi", greencount);
            telemetry.addData("Bila verde preluată în", greenBallPickedAt);
            telemetry.addData("Tag detectat", tag);


            telemetry.update();
        }

        webcamTureta.close();
    }
}


