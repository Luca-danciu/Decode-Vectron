package org.firstinspires.ftc.teamcode.OpModes;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Limelight;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.WebcamTureta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoBlueSideSmallTriangle extends OpMode {
    int stateThrow = 99;
    int stateCollect = 99;
    boolean ballsRemoved;
    ElapsedTime throwTimer;
    ElapsedTime collecttimer;

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
    public static double targetRPM1 = 4900;
    public static double targetRPM = 0;
    double measuredRPM;
    public static double pidTargetRPM = 0;

    double RPM1 , RPM2 , RPM3;

    //PIDF for turret
    private DcMotorEx turret;
    public static final double TICKS_PER_DEGREE_Turret = 6.195;
    public static final int MAX_TICKS = (int)(180 * TICKS_PER_DEGREE_Turret);

    public static double P = 0.006;
    public static double I = 0.0;
    public static double D = 0;
    public static double F = 0.1;
    private double integralTurret = 0;
    private double lastError = 0;
    public static int targetPosition = 0;

    public DcMotor launcher;

    boolean detectionActive = true;
    String detectedCase = "NONE";
    public Indexer indexer = new Indexer();
    public Outtake outtake = new Outtake();
    public Limelight limelight = new Limelight();

    //    public WebcamTureta webcam = new WebcamTureta();
    public ColorSensorIndexer colorSensorIndexer = new ColorSensorIndexer();
    public DistanceSensor distanceSensor;


    String greenBallPickedAt = "Nicio bila verde preluata";
    public String pp1 = "PickPose1";
    public String pp2 = "PickPose2";
    public String pp3 = "PickPose3";
    int purplecount = 0;
    int greencount = 0;
    public double distance;

    private Follower follower;
    private Timer pathTimer, actionTimer, nextTask , opmodeTimer;
    String tag;

    private int pathState;
    private final Pose startPose = new Pose(57, 9, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose12 = new Pose(42, 14, Math.toRadians(180));
    private final Pose scorePosePreload = new Pose(106, 16, Math.toRadians(180));
    private final Pose pickup1PosePrepare = new Pose(44, 35, Math.toRadians(180));// Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup1Pose = new Pose(17, 35, Math.toRadians(180));// Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2PosePrepare = new Pose(14,14, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(13 , 10, Math.toRadians(230));
    private final Pose park = new Pose(37 , 11, Math.toRadians(90));
    private Path scorePreload;
    public PathChain pickpose1;
    public PathChain score1;
    public PathChain pickPose2;
    public PathChain score2;
    public PathChain Park;
    public void buildPaths() {
        pickpose1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(57.000, 9.000),
                                new Pose(9.000, 23)
//                                new Pose(12.500, 8.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90))
                .addPath(new BezierLine( new Pose( 9 , 23) , new Pose(9 ,10)))
                .build();

        score1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(9, 10),
                                new Pose(34.000, 19.000),
                                new Pose(59.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(90))

                .build();

        pickPose2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.000, 12.000),
                                new Pose(75.000, 38.000),
                                new Pose(18.000, 35.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        score2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.000, 35.000),
                                new Pose(29.000, 14.000),
                                new Pose(59.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.000, 12.000),

                                new Pose(30.000, 12.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                indexer.KeepInside();
                    tag = limelight.getAprilTag();
                    greenBallPickedAt = "PickPose1";
                    if (tag != null &&
                            (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {

                        detectedCase = tag;
                        targetPosition = 100;
                        actionTimer.resetTimer();
                        outtake.Angle.setPosition(0.85);
                        targetRPM = targetRPM1;
                        stateThrow = 0;
                        setPathState(1);
                    }

                break;

            case 1:
                if (stateThrow == 99) {
                    follower.followPath(pickpose1 , 0.8 , true);
                    indexer.Colect();
                    actionTimer.resetTimer();
                    greenBallPickedAt = "PickPose2";
                    pathTimer.resetTimer();
                    stateCollect = 0;
                    setPathState(2);
                }
                break;
            case 2:

                if (!follower.isBusy() && (stateCollect == 99 || pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    indexer.KeepInside();
                    follower.followPath(score1);
                    limelight.limelight.stop();
                    targetRPM = targetRPM1;
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    stateThrow = 0;
                    setPathState(4);
                }
                break;
            case 4:
                if (stateThrow == 99){
                    follower.followPath(pickPose2  , true);
                    indexer.Colect();
                    greenBallPickedAt = "PickPose1";
                    pathTimer.resetTimer();
                    stateCollect = 0;
                    setPathState(5);
                }
                break;

            case 5:

                if (!follower.isBusy() && (stateCollect == 99 || pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    indexer.KeepInside();
                    follower.followPath(score2);
                    limelight.limelight.stop();
                    targetRPM = targetRPM1;
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    stateThrow = 0;
                    setPathState(7);
                }
                break;
            case 7:
                if (stateThrow == 99){
                    follower.followPath(Park ,0.6 , true);
                    indexer.Stop();
                    targetRPM = 0;
                    targetPosition = 0;
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        targetRPM = Math.max(0, Math.min(12000, targetRPM));

        // ✅ STOP REAL LA 0 RPM
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

        distance = distanceSensor.getDistance(DistanceUnit.MM);
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);

        switch (stateThrow) {
            // -----------------PRIMA BILA -----------------
            case 0:
                timerToSee.reset();
                targetRPM = targetRPM1;
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


                break;
            case 1:
//                    if (throwTimer.milliseconds() > 1200) {
                if (measuredRPM < (targetRPM + 150) && measuredRPM > (targetRPM -50) && throwTimer.milliseconds() > 700) {
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
                break;
            case 4:
                if ((measuredRPM < (targetRPM + 150)) && measuredRPM > (targetRPM -50) && throwTimer.milliseconds() > 500) {
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
                break;
            case 7:
                if (measuredRPM < targetRPM + 150 && measuredRPM > targetRPM -50 && throwTimer.milliseconds() > 500) {
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
                    stateThrow = 99;
                    targetRPM = 0;
                    indexer.Down();
                    Time = timerToSee.seconds();
                }
                break;
        }

        switch (stateCollect) {
            case 0: // PickPose1
                indexer.PickPose1();
                if (distance <= 60) {
                    indexer.PickPose2();
                    collecttimer.reset();
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
                    stateCollect = 99;
                }

                break;

        }

        targetPosition = wrapTicks(targetPosition);

        int currentPosition = wrapTicks(turret.getCurrentPosition());
        double power = pidf(targetPosition, currentPosition);
        power = clamp(power, -1, 1);

        turret.setPower(power);

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Caz" , tag);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("throe" , stateThrow);
        telemetry.update();
    }


    @Override
    public void init() {
        ballsRemoved = false;
        stateThrow = 99;
        stateCollect = 99;

        throwTimer  = new ElapsedTime();
        collecttimer  = new ElapsedTime();

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

        indexer.indexerinit(hardwareMap);
        outtake.outtakeinit(hardwareMap);
        limelight.limelightinit(hardwareMap);
        colorSensorIndexer.initcolorsensor(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");
        indexer.PickPose1();
        outtake.Angle.setPosition(0.76);
        indexer.Down();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        nextTask  = new Timer();
        actionTimer = new Timer();



        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        limelight.limelight.start();
        indexer.PickPose1();
        double pid = 0;
        double ff = 0;
        timer.reset();
        timerToSee.reset();

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
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
