package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
@Configurable
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
    boolean allBallsIn = false;
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

    //Set RPM Launcher
    private static final double CPR = 18.666;
    public static double kP = 0.005;
    public static double kI = 0.000007;
    public static double kD = 0.0001;
    public static double kF = 0.0001468;

    private double integral = 0;
    private double previousError = 0;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timerToSee = new ElapsedTime();
    double Time = 0;
    private double dt = 0.02;

    private int lastEncoder = 0;
    public static double targetRPM1 = 0;
    public static double targetRPM = 0;
    double measuredRPM;
    public static double pidTargetRPM = 0;

    double RPM1 , RPM2 , RPM3;

//PIDF for turret
    private DcMotorEx turret;
    public static final double TICKS_PER_DEGREE_Turret = 6.195;
    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREE_Turret);

    public static double P = 0.005;
    public static double I = 0.0;
    public static double D = 0.0002;
    public static double F = 0.05;
    private double integralTurret = 0;
    private double lastError = 0;
    public static int targetPosition = 0;

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
        turret = hardwareMap.get(DcMotorEx.class, "Tureta");

//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);


        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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

        timer.reset();
        timerToSee.reset();
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

//RPM set for launcher
            pidTargetRPM = Math.max(0, Math.min(12000, pidTargetRPM));
            if (pidTargetRPM <= 50) {
                launcher.setPower(0);
                integral = 0;
                previousError = 0;
            }else {
                double currentTime = timer.seconds();

                if (currentTime >= dt) {

                    int currentEncoder = launcher.getCurrentPosition();
                    int delta = currentEncoder - lastEncoder;

                    measuredRPM = (delta * 60.0) / (CPR * currentTime);

                    double error = pidTargetRPM - measuredRPM;

                    if (Math.abs(error) < 40) error = 0;

                    integral += error * currentTime;
                    integral = Math.max(-5000, Math.min(5000, integral));

                    double derivative = (error - previousError) / Math.max(currentTime, 0.01);

                    double output =
                            kP * error +
                                    kI * integral +
                                    kD * derivative +
                                    kF * targetRPM;

                    output = Math.max(0.0, Math.min(1.0, output));

                    launcher.setPower(output);

                    previousError = error;
                    lastEncoder = currentEncoder;
                    timer.reset();

                }
            }

            if (detectionActive) {
                    tag = webcamTureta.detectTag();
                    if (tag != null && (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {
                        detectedCase = tag;
                        gamepad1.rumble(1000);
                        newCaseDetected = true;
                    }
                }

            if (dist > 1.70 && dist < 2.5) {
                    outtake.Angle.setPosition(0.7);
                } else if (dist < 1.70 && dist > 1.50) {
                    outtake.Angle.setPosition(0.65);
                } else if (dist < 1.50 && dist > 1.00) {
                    outtake.Angle.setPosition(0.5);
                } else if (dist < 1) {
                    outtake.AngleMin();
                }


//power / dist
//                if (dist < 1.25) {
//                    targetRPM1 = 4200;
//                    pidTargetRPM = 4
//                    resetPID();
//                } else if (dist > 1.25 && dist < 1.50) {
//                    targetRPM1= 4500;
//                    resetPID();
            if (dist < 1.70) {
                    targetRPM1= 4700;
                    resetPID();
                } else if (dist > 1.70 && dist < 1.85) {
                    targetRPM1 = 5700;
                    resetPID();
                } else if (dist > 1.85 && dist < 2.5) {
                    targetRPM1 = 5700;
                    resetPID();
                } else if (dist > 2.5) {
                    outtake.Angle.setPosition(0.78);
//                    outtake.AngleMax();
                    targetRPM1 = 6000;
                    resetPID();
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
                        timerToSee.reset();
                        stopIntake = false;
                        targetRPM = targetRPM1;
                        if (targetRPM1 == 6000) {
                            pidTargetRPM = 12000;
                        } else {
                            pidTargetRPM = targetRPM1;
                        }
                        if (!colectareSelectiva) {
                            indexer.OuttakePose1();
                            throwTimer.reset();
                            stateThrow = 1;
                        } else {
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
                    if (pidTargetRPM == 12000) {
                        if (measuredRPM > (targetRPM + 900)) {
//                               if (measuredRPM > (targetRPM + 900) && throwTimer.milliseconds() > 300) {
                            RPM1 = measuredRPM;
                            indexer.Push();
                            throwTimer.reset();
                            stateThrow = 2;
                        }
                    } else {
                        if ((measuredRPM < (targetRPM + 50) && measuredRPM > (targetRPM - 50) && throwTimer.milliseconds() > 200)) {
                            indexer.Push();
                            RPM1 = measuredRPM;
                            throwTimer.reset();
                            stateThrow = 2;
                        }
                    }
                    break;

                case 2:
                    if (throwTimer.milliseconds() > 300) {
                        indexer.Down();
                        throwTimer.reset();
                        stateThrow = 3;
                    }
                    break;

                // ----------------- A DOUA BILA -----------------
                case 3:
                    if (throwTimer.milliseconds() > 200) {
                        if (!colectareSelectiva) {
                            indexer.OuttakePose2();
                            throwTimer.reset();
                            stateThrow = 4;
                        } else {
                            if (greenBallPickedAt.equals(pp1)) {
                                String caseUsed = detectedCase;

                                switch (caseUsed) {
                                    case "GPP":
                                        indexer.OuttakePose1();
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
                    if (pidTargetRPM == 12000) {
                        if (measuredRPM > (targetRPM + 900)) {
//                               if (measuredRPM > (targetRPM +900)&& throwTimer.milliseconds() > 500) {
                            RPM2 = measuredRPM;
                            indexer.Push();
                            throwTimer.reset();
                            stateThrow = 5;
                        }
                    } else {
                        if ((measuredRPM < (targetRPM + 50) && measuredRPM > (targetRPM - 50) && throwTimer.milliseconds() > 400)) {
                            indexer.Push();
                            RPM2 = measuredRPM;
                            throwTimer.reset();
                            stateThrow = 5;
                        }
                    }
                    break;
                case 5:
                    if (throwTimer.milliseconds() > 200) {
                        indexer.Down();
                        throwTimer.reset();
                        stateThrow = 6;
                    }
                    break;

                // ----------------- A TREIA BILA -----------------
                case 6:
                    if (throwTimer.milliseconds() > 200) {
                        if (!colectareSelectiva) {
                            indexer.OuttakePose3();
                            throwTimer.reset();
                            stateThrow = 7;
                        } else {
                            if (greenBallPickedAt.equals(pp1)) {
                                String caseUsed = detectedCase;

                                switch (caseUsed) {
                                    case "GPP":
                                        indexer.OuttakePose3();
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
                    if (pidTargetRPM == 12000) {
//                           if (measuredRPM > (targetRPM + 900)&& throwTimer.milliseconds() > 500) {
                        if (measuredRPM > (targetRPM + 900)) {
                            indexer.Push();
                            RPM3 = measuredRPM;
                            throwTimer.reset();
                            stateThrow = 8;
                        }
                    } else {
                        if ((measuredRPM < (targetRPM + 50) && measuredRPM > (targetRPM - 50) && throwTimer.milliseconds() > 400)) {
                            indexer.Push();
                            RPM3 = measuredRPM;
                            throwTimer.reset();
                            stateThrow = 8;
                        }
                    }
                    break;

                case 8:
                    if (throwTimer.milliseconds() > 600) {
                        if (colectareSelectiva) {
                            stateCollect = 96;
                        } else {
                            stateCollect = 0;
                        }
                        indexer.Down();
                        purplecount = 0;
                        greencount = 0;
                        stopIntake = false;
                        allBallsIn = false;
                        pidTargetRPM = 0;
                        stateThrow = 0;
                        Time = timerToSee.seconds();
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
                        break;
                }

            if (gamepad1.share) allBallsIn = true;


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
            } else if (stopIntake && collecttimer.milliseconds() > 700) {
                    indexer.Stop();
                } else {
                    indexer.KeepInside();
                }

//Unghi Tureta
            if (gamepad1.cross) outtake.AngleMin();
            if (gamepad1.circle) outtake.AngleMax();

            if (gamepad1.options) {
                    stateCollect = 0;
                    stopIntake = false;
                }

//                heading = imuIndexer.getHeading(AngleUnit.DEGREES);
//                double headingDifference = heading - initialHeading;
//
//                if (gamepad1.dpad_left) {
//                    webcamTureta.startDetection();
//                    String tagName = webcamTureta.detectTag();
//
//                    if (tagName.equals("Blue Tower")) {
//                        urmarireActivata = true;
//                        turetaHeadingFix = imuIndexer.getHeading(AngleUnit.DEGREES);
//                    }
//                }
//
//                if (gamepad1.dpad_right) {
//                    urmarireActivata = false;
//                    webcamTureta.stopDetection();
//                    outtake.turetaMotor.setTargetPosition(0);
//                    outtake.turetaMotor.setPower(1);
//                }
//
//
//                if (urmarireActivata) {
//                    double diferentaHeading = imuIndexer.getHeading(AngleUnit.DEGREES) - turetaHeadingFix;
//
//                    int targetPosition = (int) (diferentaHeading * TICKS_PER_DEGREE);
//                    outtake.turetaMotor.setTargetPosition(targetPosition);
//                    outtake.turetaMotor.setPower(1);
//                } else {
//                    outtake.turetaMotor.setTargetPosition(0);
//                }

//Tureta
            if (gamepad1.dpad_up) {
                targetPosition += 20;
            }
            if (gamepad1.dpad_down) {
                targetPosition -= 20;
            }
            targetPosition = wrapTicks(targetPosition);

            int currentPosition = wrapTicks(turret.getCurrentPosition());
            double power = pidf(targetPosition, currentPosition);
            power = clamp(power, -1, 1);

            turret.setPower(power);

            telemetry.addData("Stare camera", webcamTureta.isDetecting() ? "Pornita" : "Oprita");
            telemetry.addData("Distance", dist);
            telemetry.addData("Bile mov", purplecount);
            telemetry.addData("Bile verzi", greencount);
            telemetry.addData("Bila verde preluată în", greenBallPickedAt);
            telemetry.addData("Tag detectat", tag);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Measured RPM", measuredRPM);
            telemetry.addData("RPM1", RPM1);
            telemetry.addData("RPM2", RPM2);
            telemetry.addData("RPM3", RPM3);
            telemetry.addData("Time", Time);
            telemetry.addData("Target ticks", targetPosition);
            telemetry.addData("Current ticks", currentPosition);
            telemetry.addData("Error ticks", targetPosition - currentPosition);

            telemetry.update();
        }

        webcamTureta.close();
    }
    private void resetPID() {
        integral = 0;
        previousError = 0;
    }
    private double pidf(int target, int current) {

        int error = wrapTicks(target - current);

        integralTurret += error;
        double derivative = error - lastError;
        lastError = error;

        return (P * error)
                + (I * integralTurret)
                + (D * derivative)
                + (F * Math.signum(error));
    }
    private int wrapTicks(int ticks) {
        int range = MAX_TICKS * 2;

        while (ticks > MAX_TICKS) {
            ticks -= range;
        }
        while (ticks < -MAX_TICKS) {
            ticks += range;
        }
        return ticks;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}



