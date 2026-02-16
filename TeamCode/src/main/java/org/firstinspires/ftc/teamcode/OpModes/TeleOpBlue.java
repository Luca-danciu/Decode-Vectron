package org.firstinspires.ftc.teamcode.OpModes;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Ascent;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.IMUIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Limelight;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOpBlue extends LinearOpMode {

    //Hardware
    public DistanceSensor distanceSensor;
    public DcMotor launcher;
    public Drivetrain drivetrain = new Drivetrain();
    public Indexer indexer = new Indexer();
    public Outtake outtake = new Outtake();
    public IMUIndexer imuIndexer = new IMUIndexer();
    public ColorSensorIndexer colorSensorIndexer = new ColorSensorIndexer();
    public Limelight limelight = new Limelight();
    public Ascent ascent = new Ascent();
    private Follower follower;

    DigitalChannel ledRedD;
    DigitalChannel ledGreenD;
    DigitalChannel ledRedS;
    DigitalChannel ledGreenS;


    //valori
    double distance = 0;
    double powerLauncher = 0.9;
    int purplecount = 0;
    int greencount = 0;
    double Time = 0;
    public int towerX = 14;
    public int towerY = 138;
//143

    //Tureta
    static final double TICKS_PER_DEGREE = 6.071111;
    double dx = 0;
    double dy = 0;
    double headingRobot = 0;
    double alpha = 0;
    double headingTuretaGrade;
    double targetTicks = 0;
    double powerT;
    private double integralT = 0;
    private double lastErrorT = 0;
    public static double kPT = 0.005;
    public static double kIT = 0.0;
    public static double kDT = 0.0002;
    public static double kFT = 0.05;
    public static final int MAX_TICKST = (int)(180 * TICKS_PER_DEGREE);
    public static final int MIN_TICKST = -MAX_TICKST;
    private final Pose START_POSE = new Pose(57, 113, Math.toRadians(180)); // Exemplu: poziție start + heading 90° (spre nord)
//    private final Pose START_POSE = new Pose(32, 12, Math.toRadians(180)); // Exemplu: poziție start + heading 90° (spre nord)


    static final double TICKS_PER_DEGREET = 6.533;
    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREET);
    public static final int MIN_TICKS = -MAX_TICKS;

    public static double P = 0.006;
    public static double I = 0.0;
    public static double D = 0;
    public static double F = 0.1;
    private double integralTurret = 0;
    private double lastError = 0;
    public static int targetPosition = 0;

    public boolean TuretaToZero = false;

    //verificari
    boolean TakeOUT = false;
    boolean newCaseDetected = false;
    boolean allBallsIn = false;
    boolean colectareSelectiva = false;
    boolean stopIntake = false;
    boolean ballsRemoved = false;

    //Timer
    private final ElapsedTime timerToSee = new ElapsedTime();
    private final ElapsedTime TakeGreenBallOut = new ElapsedTime();
    private final ElapsedTime TakePurpleBallOut = new ElapsedTime();



    //Strings
    String detectedCase = "NONE";
    public String pp1 = "PickPose1";
    public String pp2 = "PickPose2";
    public String pp3 = "PickPose3";
    String greenBallPickedAt = "Nicio bila verde preluata";
    ColorSensorIndexer.DetectedColor detectedColor1, detectedColor2, detectedColor3;
    //Pidf aruncare
    private static final double CPR = 18.666;

    // ✅ PIDF REGLABIL DIN PANELS (OBLIGATORIU public static)
    public static double kP = 0.003;
    public static double kI = 0.000007;
    public static double kD = 0.00007;
    public static double kF = 0.0001468;
    public static double servo = 0.28;

    private double integral = 0;
    private double previousError = 0;

    private ElapsedTime timer = new ElapsedTime();
    private double dt = 0.02;

    private int lastEncoder = 0;
    public static double targetRPM = 0;
    public static double getTargetRPM = 0;

    public static double targetR1 = 3400;
    public static double targetR2 = 3600;
    public static double targetR3 = 4200;
    public static double targetR4 = 4800;

    double measuredRPM = 0;
    boolean resumeFromLast = false;
    boolean ridic = false;
    int turretHoldTicks = 0;

    private boolean pathFinished = false;
    private boolean automatedDrive;
    private Supplier<PathChain> parcare;
    public static double DR= 1;
    public static double SR = 1;

    public void runOpMode() throws InterruptedException {

        int stateThrow = 0;
        int stateCollect = 0;
        int statePark = 0;

        int stateTureta = 0;
        ElapsedTime statetureta = new ElapsedTime();

        ElapsedTime throwTimer = new ElapsedTime();
        ElapsedTime stopCollect = new ElapsedTime();
        ElapsedTime collecttimer = new ElapsedTime();
        ElapsedTime DpadPressed = new ElapsedTime();

        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");
        launcher = hardwareMap.get(DcMotor.class, "Launcher");
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "Tureta");

//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        ledRedD = hardwareMap.get(DigitalChannel.class, "ledRedD");
        ledGreenD = hardwareMap.get(DigitalChannel.class, "ledGreenD");
        ledRedS = hardwareMap.get(DigitalChannel.class, "ledRedS");
        ledGreenS = hardwareMap.get(DigitalChannel.class, "ledGreenS");

        ledRedD.setMode(DigitalChannel.Mode.OUTPUT);
        ledGreenD.setMode(DigitalChannel.Mode.OUTPUT);
        ledRedS.setMode(DigitalChannel.Mode.OUTPUT);
        ledGreenS.setMode(DigitalChannel.Mode.OUTPUT);

        ledRedD.setState(false);
        ledGreenD.setState(true);
        ledRedS.setState(false);
        ledGreenS.setState(true);

        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        colorSensorIndexer.initcolorsensor(hardwareMap);
        indexer.indexerinit(hardwareMap);
        outtake.outtakeinit(hardwareMap);
        imuIndexer.init(hardwareMap);
        limelight.limelightinit(hardwareMap);
        drivetrain.drivetraininit(hardwareMap);
        ascent.ascentinit(hardwareMap);

        timerToSee.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose( START_POSE );
        parcare = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(111, 29.5))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                .build();

        waitForStart();

        indexer.PickPose1();
        indexer.Down();
        limelight.limelight.start();
        DpadPressed.reset();
        statetureta.reset();
        targetRPM = 0;
        ascent.Prins();


        while (opModeIsActive() && !isStopRequested()) {

            follower.update();
            if (gamepad1.cross && !automatedDrive) {
                follower.followPath(parcare.get());
                automatedDrive = true;
                pathFinished = false;
            }
            if (!automatedDrive) {
                if (Math.abs(gamepad1.left_stick_y) < 0.05 &&
                        Math.abs(gamepad1.left_stick_x) < 0.05 &&
                        Math.abs(gamepad1.right_stick_x) < 0.05 &&
                        Math.abs(gamepad1.left_stick_y) > -0.05 &&
                        Math.abs(gamepad1.left_stick_x) > -0.05 &&
                        Math.abs(gamepad1.right_stick_x) > -0.05) {

                    drivetrain.activeBrake();
                } else {
                    drivetrain.drive(-gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x);
                }
            }
            if (automatedDrive && !follower.isBusy() && !pathFinished) {
                pathFinished = true;

                // Opreste complet robotul
                drivetrain.drive(0, 0, 0);
                drivetrain.activeBrake();
            }
            if (pathFinished && gamepad1.square) {
                automatedDrive = false;
                pathFinished = false;
            }

            distance = distanceSensor.getDistance(DistanceUnit.MM);
            if (allBallsIn){
                ledRedD.setState(true);
                ledGreenD.setState(false);
                ledRedS.setState(true);
                ledGreenS.setState(false);
            }else {
                ledRedD.setState(false);
                ledGreenD.setState(true);
                ledRedS.setState(false);
                ledGreenS.setState(true);
            }

//Tureta
            if (gamepad1.left_bumper){
                follower.setPose( new Pose(39 , 140 ,Math.toRadians(180)));
            }
            switch (stateTureta){
                case 0:
                    if (resumeFromLast){
                        targetTicks = turretHoldTicks;
                        resumeFromLast = false;
                    }else {
                        double dx = towerX - follower.getPose().getX();
                        double dy = towerY - follower.getPose().getY();

                        double alpha1 = Math.toDegrees(Math.atan2(dy, dx));
                        double headingRobot = Math.toDegrees(follower.getHeading());

                        double headingTuretaGrade = alpha1 - headingRobot;
                        if (headingTuretaGrade > 180) {
                            headingTuretaGrade = headingTuretaGrade - 360;
                        }
                        if (headingTuretaGrade < -180) {
                            headingTuretaGrade = headingTuretaGrade + 360;

                        }
                        int targetTicks = (int) (headingTuretaGrade * TICKS_PER_DEGREE);
                        int currentTicks = turret.getCurrentPosition();
                        int error = targetTicks - currentTicks;
                        if (currentTicks >= MAX_TICKST && powerT > 0) powerT = 0;
                        if (currentTicks <= MIN_TICKS && powerT < 0) powerT = 0;
                        double powerT =
                                kPT * error +
                                        kDT * (error - lastErrorT);

                        lastError = error;
                        powerT = Math.max(-1, Math.min(1, powerT));
                        turret.setPower(powerT);
//                    follower.update();
                        if (gamepad1.share && statetureta.milliseconds() > 400) {
                            statetureta.reset();
                            stateTureta = 1;
                        }
                    }
                    break;

                case 1:
                    if (statetureta.milliseconds() > 400){
                        LLResult result = limelight.limelight.getLatestResult();
                        if ((result == null || !result.isValid()) && !TuretaToZero) {
                            turret.setPower(0);
                        }
                        List<LLResultTypes.FiducialResult> tags = null;
                        if (result != null) {
                            tags = result.getFiducialResults();
                        }
                        if ((tags == null || tags.isEmpty() ) && !TuretaToZero) {
                            turret.setPower(0);
                        }
                        LLResultTypes.FiducialResult targetTag = null;
                        if (tags != null) {
                            for (LLResultTypes.FiducialResult t : tags) {
                                if (t.getFiducialId() == 20) {
                                    targetTag = t;
                                    break;
                                }
                            }
                        }
                        if (targetTag == null && !TuretaToZero) {
                            turret.setPower(0);
                        }
                        double tx = 0;
                        if (targetTag != null) {
                            tx = targetTag.getTargetXDegrees();
                        }
                        if (Math.abs(tx) < 1.0 && !TuretaToZero) {
                            turret.setPower(0);
                        }
                        double power = tx * 0.03;
                        int pos = turret.getCurrentPosition();
                        if ((pos >= MAX_TICKS && power > 0) || (pos <= MIN_TICKS && power < 0)) {
                            power = 0;
                        }
                        turret.setPower(power);
                        if (gamepad1.right_bumper) {
                            targetPosition = 0;
                            targetPosition = wrapTicks(targetPosition);
                            TuretaToZero = true;
                            int currentPosition = wrapTicks(turret.getCurrentPosition());
                            power = pidf(targetPosition, currentPosition);
                            power = clamp(power, -1, 1);
                            turret.setPower(power);
                        } else {
                            power = Math.max(-0.6, Math.min(0.6, power));
                            TuretaToZero = false;
                        }
                        if (gamepad1.share) {
                            turretHoldTicks = turret.getCurrentPosition();
                            resumeFromLast = true;
                            statetureta.reset();
                            stateTureta = 0;
                        }
                    }
                    break;
            }

//            switch (statePark){
//                case 0:
//                    if (gamepad1.dpad_left){
//                        DpadPressed.reset();
//                        statePark = 1;
//
//                    }
//                    break;
//                case 1:
//                    if (gamepad1.dpad_left && DpadPressed.milliseconds() < 1000){
//                        ascent.Eliberat();
//                        ridic = true;
//                        indexer.Stop();
//                        targetRPM = 0;
//                        limelight.limelight.stop();
//                        statePark = 2;
//                    }
//                    if (DpadPressed.milliseconds() > 300){
//                        statePark = 0;
//                    }
//                    break;
//
//            }
            if (gamepad1.dpad_up){
                ascent.Eliberat();
                ridic = true;
            }
            if (gamepad1.dpad_left){
                ascent.Prins();
            }
            if (ridic){
                targetPosition = 0;
                targetRPM = 0;
                indexer.Stop();
                drivetrain.activeBrake();

            }
            if (gamepad1.dpad_down){
                ascent.Ridicare(DR,SR);
            }

            else {
                ascent.Ridicare(0,0);
            }
//Detectie tureta ATTracking

            limelight.limelight.start();

            String tag = limelight.getAprilTag();
//            double dist = limelight.getAprilTagDistance();
            double dx = towerX - follower.getPose().getX();
            double dy = towerY - follower.getPose().getY();
            double dist = Math.sqrt((dx*dx) + (dy*dy));

//Detectie Tag
            if (tag != null && (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {
                detectedCase = tag;
                newCaseDetected = true;
            }

//Unghi outtake

            if (dist < 1.2){
                outtake.Angle.setPosition(0.5);
                getTargetRPM = 3200;
            }else if (dist > 1.2 && dist <1.9){
                outtake.Angle.setPosition(0.67);
                getTargetRPM = 3400;
            }else if (dist > 1.9 && dist< 3){
                outtake.Angle.setPosition(0.7);
                getTargetRPM = 3500;
            }else if (dist > 3 && dist< 4.5){
                outtake.Angle.setPosition(0.7);
                getTargetRPM = 3700;
            }else if (dist > 4.5 && dist< 8){
                outtake.Angle.setPosition(0.84);
                getTargetRPM = 4000;
            }else if (dist > 8){
                outtake.Angle.setPosition(0.84);
                getTargetRPM = 4800;
            }



//Throw
            switch (stateThrow) {
                // -----------------PRIMA BILA -----------------
                case 0:
                    if (gamepad1.triangle ) {
                        TakeOUT = false;
                        timerToSee.reset();
                        stopIntake = false;
                        targetRPM = getTargetRPM;
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
//                    if (throwTimer.milliseconds() > 1200) {
                    if (measuredRPM < (targetRPM + 100) && measuredRPM > (targetRPM -100) && throwTimer.milliseconds() > 400) {
                        indexer.Push();
                        throwTimer.reset();
                        stateThrow = 2;
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
                    if ((measuredRPM < (targetRPM + 100)) && measuredRPM > (targetRPM -100) && throwTimer.milliseconds() > 400) {
                        indexer.Push();
                        throwTimer.reset();
                        stateThrow = 5;
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
                    if (measuredRPM < targetRPM + 100 && measuredRPM > targetRPM -100 && throwTimer.milliseconds() > 400) {
                        indexer.Push();
                        throwTimer.reset();
                        stateThrow = 8;
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
                        targetRPM = 0;
                        indexer.Down();
                        purplecount = 0;
                        greencount = 0;
                        stopIntake = false;
                        allBallsIn = false;
                        stateThrow = 0;
                        Time = timerToSee.seconds();
                        greenBallPickedAt = "Nicio bila verde preluata";
                    }
                    break;
            }

//Collect
            switch (stateCollect) {
                case 0: // PickPose1
                    indexer.PickPose1();
                    if (distance <= 70) {
                        indexer.PickPose2();
                        collecttimer.reset();
                        ballsRemoved = false;
                        stateCollect = 1;
                    }
                    break;

                case 1:
                    if (distance <= 70 && collecttimer.milliseconds() > 600) {
                        indexer.PickPose3();
                        collecttimer.reset();
                        stateCollect = 2;
                    }
                    break;

                case 2: // PickPose3
                    if (distance <= 70 && collecttimer.milliseconds() > 300) {
                        collecttimer.reset();
                        indexer.OuttakePose1();
                        targetRPM =getTargetRPM;
                        allBallsIn = true;
                        stopIntake = true;
                        gamepad1.rumble(300);
                        stateCollect = 3;

                    }
                    break;

                case 3: // PickPose3
                    if (collecttimer.milliseconds() > 300) {
                        collecttimer.reset();
                        TakeOUT = true;
                        stateCollect = 4;

                    }
                    break;

                case 4:
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
                    if (distance <= 70) {
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
                    if (distance <= 70 && collecttimer.milliseconds() > 600) {
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
                    if (distance <= 70 && collecttimer.milliseconds() > 300) {
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

            targetRPM = Math.max(0, Math.min(12000, targetRPM));

            if (targetRPM <= 50) {
                launcher.setPower(0);
                integral = 0;
                previousError = 0;


            }

            double currentTime = timer.seconds();
            if (currentTime >= dt) {

                int currentEncoder = launcher.getCurrentPosition();
                int delta = currentEncoder - lastEncoder;

                measuredRPM = (delta * 60.0) / (CPR * currentTime);

                double error = targetRPM - measuredRPM;

                // ✅ DEADZONE ANTI TREPIDAT
                if (Math.abs(error) < 40) error = 0;

                // ✅ INTEGRAL CU LIMITARE (ANTI WIND-UP)
                integral += error * currentTime;
                integral = Math.max(-5000, Math.min(5000, integral));

                // ✅ DERIVATĂ FILTRATĂ
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
            if (gamepad1.left_trigger > 0 && !ridic) {
                indexer.KeepInside();
            } else if (gamepad1.right_trigger > 0 && !ridic) {
                indexer.TakeOut();
            }else if(TakeOUT && !ridic){
                indexer.TakeOutBit();
            }else if (TakeGreenBallOut.milliseconds() < 400 && !ridic) {
                indexer.TakeOut();
                greencount = 1;
            } else if (TakePurpleBallOut.milliseconds() < 400 && !ridic) {
                indexer.TakeOut();
                purplecount = 2;
            }else if ((stopIntake && collecttimer.milliseconds() > 700) || (ridic)) {
                indexer.Stop();
            } else if (!ridic){
                indexer.Colect();
            }

//Reset Collect
            if (gamepad1.options) {
                stateCollect = 0;
                TakeOUT = false;
                stopIntake = false;
            }

//Power Launcher
            if (gamepad1.dpad_left && DpadPressed.milliseconds() > 200) {
                DpadPressed.reset();
                powerLauncher -= 0.1;
            }
            if (gamepad1.dpad_right && DpadPressed.milliseconds() > 200) {
                DpadPressed.reset();
                powerLauncher += 0.1;

            }

            telemetry.addData("Stare camera", limelight.limelight.isRunning() ? "Pornita" : "Oprita");
            telemetry.addData("Distanta", dist);
            telemetry.addData("statethrow", stateThrow);
            telemetry.addData("Target RPM", getTargetRPM);
            telemetry.addData("Measured RPM", measuredRPM);
            telemetry.addData("TimeThrow", Time);
            telemetry.addData("Bile mov", purplecount);
            telemetry.addData("Bile verzi", greencount);
//            telemetry.addData("TT", throwTimer.milliseconds());
            telemetry.addData("Tureta", statePark);
            telemetry.update();

        }
        limelight.limelight.stop();
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

