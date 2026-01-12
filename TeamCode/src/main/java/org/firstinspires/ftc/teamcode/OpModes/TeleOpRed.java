package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.Hardware.Limelight;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.WebcamTureta;

import java.util.List;

@TeleOp(name = "TeleOpRed")
@Configurable
public class TeleOpRed extends LinearOpMode {

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
    //    public WebcamTureta webcamTureta = new WebcamTureta();
    public IMUIndexer imuIndexer = new IMUIndexer();
    public ColorSensorIndexer colorSensorIndexer = new ColorSensorIndexer();
    public Limelight limelight = new Limelight();

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

    boolean LaunchFromFar = false;
    public double powerLauncher = 0.9;

    //PIDF for turret\
    private DcMotorEx turret;
    static final double TICKS_PER_DEGREET = 6.195;

    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREET);
    public static final int MIN_TICKS = -MAX_TICKS;

    // ================= PIDF =================
    public static double P = 0.005;
    public static double I = 0.0;
    public static double D = 0.0002;
    public static double F = 0.05;

    private double integralT = 0;
    private double lastErrorT = 0;
    static final int TARGET_TAG_ID = 24;
    private boolean targetVisible = false;
    int desiredTicks = 0;
    boolean returnToZero = false;
    static final int TURRET_ZERO_TICKS = 0;
    static final int ZERO_DEADBAND = 10;

//    private DcMotorEx turret;
//    public static final double TICKS_PER_DEGREE_Turret = 6.195;
//    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREE_Turret);
//
//    public static double P = 0.006;
//    public static double I = 0.0;
//    public static double D = 0;
//    public static double F = 0.1;
//    private double integralTurret = 0;
//    private double lastError = 0;
//    public static int targetPosition = 0;

    public static double interval1 = 0.552;
    public static double interval2 = 0.7;
    public static double interval3 = 0.73;
    public static double interval4 = 0.78;

    public void runOpMode() throws InterruptedException {

        int stateThrow = 0;
        int stateCollect = 0;

        ElapsedTime throwTimer = new ElapsedTime();
        ElapsedTime collecttimer = new ElapsedTime();
        ElapsedTime TakeGreenBallOut = new ElapsedTime();
        ElapsedTime TakePurpleBallOut = new ElapsedTime();
        ElapsedTime DpadPressed = new ElapsedTime();

        RearLeft = hardwareMap.get(DcMotor.class, "RL");
        RearRight = hardwareMap.get(DcMotor.class, "RR");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        FrontLeft = hardwareMap.get(DcMotor.class, "FL");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");
        launcher = hardwareMap.get(DcMotor.class, "Launcher");
        turret = hardwareMap.get(DcMotorEx.class, "Tureta");

//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        colorSensorIndexer.initcolorsensor(hardwareMap);
        indexer.indexerinit(hardwareMap);
        outtake.outtakeinit(hardwareMap);
//        webcamTureta.initwebcam(hardwareMap);
        imuIndexer.init(hardwareMap);
        limelight.limelightinit(hardwareMap);

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

        DpadPressed.reset();
        initialHeading = imuIndexer.getHeading(AngleUnit.DEGREES);
        indexer.PickPose1();
        double pid = 0;
        double ff = 0;
        indexer.Down();
        limelight.limelight.start();

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

//            String tag = webcamTureta.detectTag();
//            double dist = webcamTureta.getDistanceMeters();
            String tag = limelight.getAprilTag();
            double dist = limelight.getAprilTagDistance();

//RPM set for launcher
//            pidTargetRPM = Math.max(0, Math.min(12000, pidTargetRPM));
//            if (pidTargetRPM <= 50) {
//                launcher.setPower(0);
//                integral = 0;
//                previousError = 0;
//            }else {
//                double currentTime = timer.seconds();
//
//                if (currentTime >= dt) {
//
//                    int currentEncoder = launcher.getCurrentPosition();
//                    int delta = currentEncoder - lastEncoder;
//
//                    measuredRPM = (delta * 60.0) / (CPR * currentTime);
//
//                    double error = pidTargetRPM - measuredRPM;
//
//                    if (Math.abs(error) < 40) error = 0;
//
//                    integral += error * currentTime;
//                    integral = Math.max(-5000, Math.min(5000, integral));
//
//                    double derivative = (error - previousError) / Math.max(currentTime, 0.01);
//
//                    double output =
//                            kP * error +
//                                    kI * integral +
//                                    kD * derivative +
//                                    kF * targetRPM;
//
//                    output = Math.max(0.0, Math.min(1.0, output));
//
//                    launcher.setPower(output);
//
//                    previousError = error;
//                    lastEncoder = currentEncoder;
//                    timer.reset();
//
//                }
//            }

//            if (detectionActive) {
//                    tag = webcamTureta.detectTag();

            LLResult result = limelight.limelight.getLatestResult();
            if (result == null || !result.isValid()) {
                turret.setPower(0);
//                continue;
            }

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags == null || tags.isEmpty()) {
                turret.setPower(0);
//                continue;
            }

            LLResultTypes.FiducialResult targetTag = null;
            for (LLResultTypes.FiducialResult t : tags) {
                if (t.getFiducialId() == 24) {
                    targetTag = t;
                    break;
                }
            }

            if (targetTag == null) {
                turret.setPower(0);
//                continue;
            }

            double tx = 0;
            if (targetTag != null) {
                tx = targetTag.getTargetXDegrees();
                returnToZero = false;
            }

            if (Math.abs(tx) < 1.0) {
                turret.setPower(0);
//                continue;
            }

            double power = tx * 0.03;
            power = Math.max(-0.6, Math.min(0.6, power));

            int pos = turret.getCurrentPosition();
            if ((pos >= MAX_TICKS && power > 0) ||
                    (pos <= MIN_TICKS && power < 0)) {
                power = 0;
            }
//            if (gamepad1.cross) {
//                returnToZero = true;
//            }
//            if (returnToZero) {
//
//                int current = turret.getCurrentPosition();
//                int error = TURRET_ZERO_TICKS - current;
//
//                if (Math.abs(error) < ZERO_DEADBAND) {
//                    turret.setPower(0);
//                    returnToZero = false; // a ajuns la zero
//                } else {
//                    power = error * 0.005; // P simplu
//                    power = Math.max(-0.5, Math.min(0.5, power));
//                    turret.setPower(power);
//                }
//
//            }


            turret.setPower(power);



            if (tag != null && (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {
                detectedCase = tag;
//                        gamepad1.rumble(500);
                newCaseDetected = true;
            }

            if (dist > 2.47 && dist < 5.38) {
                outtake.Angle.setPosition(interval3);
            } else if (dist < 2.47 && dist > 1.91) {
                outtake.Angle.setPosition(interval2);
            } else if (dist < 1.91 && dist > 0.87) {
                outtake.Angle.setPosition(interval1);
            } else if (dist < 0.87) {
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
//            if (dist < 1.91) {
//                    targetRPM1= 4700;
//                    resetPID();
//                } else if (dist > 1.91 && dist < 2.47) {
//                    targetRPM1 = 5700;
//                    resetPID();
//                } else if (dist > 2.47 && dist < 5.38) {
//                    targetRPM1 = 5700;
//                    resetPID();
//                } else if (dist > 5.38) {
//                outtake.Angle.setPosition(interval4);             outtake.AngleMax();
//                targetRPM1 = 6000;
//                resetPID();
//            }
//
//
//Porneste detectarea
//            if (gamepad1.right_bumper) {
//                    detectionActive = true;
//                    webcamTureta.startDetection();
//                }
//Opreste detectarea
//            else {
//                    detectionActive = false;
//                    webcamTureta.stopDetection();
//                }


//Aruncare
            switch (stateThrow) {
                // ----------------- PRIMA BILA -----------------
                case 0:
                    if (gamepad1.triangle && allBallsIn) {
                        timerToSee.reset();
                        stopIntake = false;
                        launcher.setPower(powerLauncher);
//                        targetRPM = targetRPM1;
//                        if (targetRPM1 == 6000) {
//                            pidTargetRPM = 12000;
//                        } else {
//                            pidTargetRPM = targetRPM1;
//                        }
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
//                        if ((measuredRPM < (targetRPM + 50) && measuredRPM > (targetRPM - 50) && throwTimer.milliseconds() > 200)) {
                        if (throwTimer.milliseconds() > 1200){
                            indexer.Push();
                            RPM1 = measuredRPM;
                            throwTimer.reset();
                            stateThrow = 2;
                        }
                    }
                    break;

                case 2:
                    if (throwTimer.milliseconds() > 200) {
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
                        if (throwTimer.milliseconds() > 500) {
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
                        if (throwTimer.milliseconds() > 500) {
                            indexer.Push();
                            RPM3 = measuredRPM;
                            throwTimer.reset();
                            stateThrow = 8;
                        }
                    }
                    break;

                case 8:
                    if (throwTimer.milliseconds() > 200) {
                        indexer.Down();
                        stateThrow = 9;
                    }
                    break;
                case 9:
                    if (throwTimer.milliseconds() > 400) {
                        if (colectareSelectiva) {
                            stateCollect = 96;
                        } else {
                            stateCollect = 0;
                        }
                        LaunchFromFar = false;
                        launcher.setPower(0);
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
                        gamepad1.rumble(300);
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
                            gamepad1.rumble(200);
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


            if (gamepad1.options) {
                stateCollect = 0;
                stopIntake = false;
            }
            if (gamepad1.dpad_left && DpadPressed.milliseconds() > 200){
                DpadPressed.reset();
                powerLauncher -= 0.1;
            }
            if (gamepad1.dpad_right && DpadPressed.milliseconds() > 200){
                DpadPressed.reset();
                powerLauncher += 0.1;

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
//            if (gamepad1.dpad_up) {
//                targetPosition += 20;
//            }
//            if (gamepad1.dpad_down) {
//                targetPosition -= 20;
//            }
//            targetPosition = wrapTicks(targetPosition);
//
//            int currentPosition = wrapTicks(turret.getCurrentPosition());
//            double power = pidf(targetPosition, currentPosition);
//            power = clamp(power, -1, 1);

//            turret.setPower(power);

            telemetry.addData("Stare camera", limelight.limelight.isRunning() ? "Pornita" : "Oprita");
            telemetry.addData("Distance", dist);
            telemetry.addData("Bile mov", purplecount);
            telemetry.addData("Bile verzi", greencount);
//            telemetry.addData("Bila verde preluată în", greenBallPickedAt);
            telemetry.addData("Tag detectat", tag);
//            telemetry.addData("Target RPM", targetRPM);
//            telemetry.addData("Measured RPM", measuredRPM);
//            telemetry.addData("RPM1", RPM1);
//            telemetry.addData("RPM2", RPM2);
//            telemetry.addData("RPM3", RPM3);
//            telemetry.addData("Time", Time);
//            telemetry.addData("Target ticks", targetPosition);
//            telemetry.addData("Current ticks", currentPosition);
//            telemetry.addData("Error ticks", targetPosition - currentPosition);
            telemetry.addData("PowerLauncher"  , powerLauncher);


            telemetry.update();
        }
        limelight.limelight.stop();
//        webcamTureta.close();
    }
    private void resetPID() {
        integral = 0;
        previousError = 0;
    }
    private double pidf(int target, int current) {

        int error = wrapTicks(target - current);

        integralT += error;
        double derivative = error - lastErrorT;
        lastErrorT = error;

        return (kP * error)
                + (kI * integralT)
                + (kD * derivative)
                + (kF * Math.signum(error));
    }
    private int wrapTicks(int ticks) {
        int range = MAX_TICKS * 2;

        while (ticks > MAX_TICKS) ticks -= range;
        while (ticks < -MAX_TICKS) ticks += range;

        return ticks;
    }
    private void stopTurret() {
        turret.setPower(0);
        integralT = 0;
        lastErrorT = 0;
        targetVisible = false;
    }
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}



