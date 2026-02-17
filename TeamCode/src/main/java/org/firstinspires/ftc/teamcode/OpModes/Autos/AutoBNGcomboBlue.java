package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.OpmodesConstants.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Limelight;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoBNGcomboBlue extends OpMode {
    int stateThrow = 99;
    int stateCollect = 99;
    boolean ballsRemoved;
    ElapsedTime throwTimer;
    ElapsedTime collecttimer;

    private double integral = 0;
    private double previousError = 0;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timerToSee = new ElapsedTime();
    double Time = 0;

    private int lastEncoder = 0;
    public static double targetRPM1 = AUTO_LAUNCHER_RPM_BNG;
    public static double targetRPM2 = AUTO_LAUNCHER_RPM_BNG_SECOND;
    public static double targetRPM = 0;
    double measuredRPM;
    public static double pidTargetRPM = 0;

    double RPM1 , RPM2 , RPM3;

    //PIDF for turret
    private DcMotorEx turret;
    public static final int MAX_TICKS = (int)(180 * TURRET_TICKS_PER_DEGREE_LIMELIGHT);
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
    private final Pose startPose = new Pose(63, 8, Math.toRadians(90)); // Start Pose of our robot.
    public PathChain scorePreload;
    public PathChain pick1;
    public PathChain score1;
    public PathChain pick2;
    public PathChain pick2final;
    public PathChain score2;
    public PathChain pick3;
    public PathChain score3;
    public void buildPaths() {


        scorePreload = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(63.000, 8.000),

                                new Pose(61.000, 10.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))

                .build();

        pick1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(61.000, 10.000),
                                new Pose(50.000, 40.000),
                                new Pose(14.000, 36.000)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        score1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 36.000),

                                new Pose(60.000, 12.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pick2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 12.000),

                                new Pose(13.000, 13.000)
                        )
                ).setTangentHeadingInterpolation()

                        .addPath(
                        new BezierCurve(
                new Pose(14.000, 12.000),
                new Pose(23.000, 10.000),
                new Pose(14.000, 9.000)
        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();


        score2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.000, 9.000),

                                new Pose(60.000, 11.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        pick3 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                new Pose(60.000, 11.000),
//
//                                new Pose(13.000, 12.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                .addPath(
//                        new BezierCurve(
//                                new Pose(13.000, 12.000),
//                                new Pose(23.000, 10.000),
//                                new Pose(13.000, 9.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(60, 11),
                                new Pose(40, 12)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();


        score3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.000, 9.000),

                                new Pose(60.000, 11.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                TURRET_POS_F = AUTO_TURRET_POS_F_MOVING;
                indexer.KeepInside();
                follower.followPath(scorePreload);
                targetPosition = -20;
                actionTimer.resetTimer();
                outtake.Angle.setPosition(OUTTAKE_ANGLE_FAR);
                targetRPM = targetRPM1;
//                stateThrow = 0;
                setPathState(1);

                break;

            case 1:
                if (!follower.isBusy()) {
                    TURRET_POS_F = AUTO_TURRET_POS_F_HOLDING;
                    stateThrow = 0;
                    setPathState(2);
                }
                break;
            case 2:

                if (stateThrow == 99 ) {
                    indexer.Colect();
                    stateCollect = 0;
                    follower.followPath(pick1);
                    targetPosition = -240;
                    targetRPM = targetRPM1;


                    setPathState(3);
                }
                break;
            case 3:
                if (follower.isBusy()){
                    actionTimer.resetTimer();
                }
                if (actionTimer.getElapsedTimeSeconds() > AUTO_BNG_SCORE_DELAY_SEC ){
                    indexer.KeepInside();
                    targetRPM = targetRPM1;
                    follower.followPath(score1, AUTO_PATH_SPEED_FAST , true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    stateThrow = 0 ;
                    setPathState(5);
                }
                break;

            case 5:

                if (stateThrow == 99) {
                    follower.followPath(pick2);
                    targetRPM = targetRPM1;

                    targetPosition = -420;

                    stateCollect = 0;
                    indexer.Colect();
                    setPathState(6);
                }
                break;
            case 6:

                if (!follower.isBusy() || stateCollect == 99){
                    targetRPM = targetRPM1;
                    follower.followPath(score2, AUTO_PATH_SPEED_FAST, true);
                    indexer.KeepInside();
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    stateThrow = 0;
                    setPathState(11);
                }
                break;
//            case 8:
//                if (stateThrow == 99){
//                    indexer.Colect();
//                    stateCollect = 0;
//                    follower.followPath(pick3);
//                    targetRPM = targetRPM1;
//
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if (!follower.isBusy() || stateCollect == 99){
//                    targetRPM = targetRPM2;
//                    follower.followPath(score3);
//                    indexer.KeepInside();
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if (!follower.isBusy()){
//                    stateThrow = 0;
//                    setPathState(11);
//                }
//                break;
            case 11:
                if (stateThrow == 99){
                    follower.followPath(pick3);
                    targetPosition = 0;
                    indexer.Stop();
                    targetRPM = 0;
                }

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        targetRPM = Math.max(0, Math.min(LAUNCHER_MAX_RPM, targetRPM));

        if (targetRPM <= LAUNCHER_MIN_ACTIVE_RPM) {
            launcher.setPower(0);
            integral = 0;
            previousError = 0;


        }

        double currentTime = timer.seconds();
        if (currentTime >= LAUNCHER_DT) {

            int currentEncoder = launcher.getCurrentPosition();
            int delta = currentEncoder - lastEncoder;

            measuredRPM = (delta * 60.0) / (LAUNCHER_CPR * currentTime);

            double error = targetRPM - measuredRPM;

            if (Math.abs(error) < LAUNCHER_ERROR_DEADZONE) error = 0;

            integral += error * currentTime;
            integral = Math.max(-LAUNCHER_INTEGRAL_LIMIT, Math.min(LAUNCHER_INTEGRAL_LIMIT, integral));

            // ✅ DERIVATĂ FILTRATĂ
            double derivative = (error - previousError) / Math.max(currentTime, LAUNCHER_PID_DERIVATIVE_TIME_MIN);

            double output =
                    LAUNCHER_KP * error +
                            LAUNCHER_KI * integral +
                            LAUNCHER_KD * derivative +
                            LAUNCHER_KF * targetRPM;

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
                indexer.KeepInside();
                targetRPM = targetRPM1;
                indexer.OuttakePose1();
//                if (greenBallPickedAt.equals(pp1)) {
//                    String caseUsed = detectedCase;
//                    switch (caseUsed) {
//                        case "GPP":
//                            indexer.OuttakePose2();
//                            break;
//                        case "PGP":
//                            indexer.OuttakePose3();
//                            break;
//                        case "PPG":
//                            indexer.OuttakePose3();
//                            break;
//                        default:
//                            indexer.OuttakePose1();
//                            break;
//                    }
//                }
//                if (greenBallPickedAt.equals(pp2)) {
//                    String caseUsed = detectedCase;
//
//                    switch (caseUsed) {
//                        case "GPP":
//                            indexer.OuttakePose3();
//                            break;
//                        case "PGP":
//                            indexer.OuttakePose2();
//                            break;
//                        case "PPG":
//                            indexer.OuttakePose2();
//                            break;
//                        default:
//                            indexer.OuttakePose1();
//                            break;
//                    }
//                }
//                if (greenBallPickedAt.equals(pp3)) {
//                    String caseUsed = detectedCase;
//
//                    switch (caseUsed) {
//                        case "GPP":
//                            indexer.OuttakePose1();
//                            break;
//                        case "PGP":
//                            indexer.OuttakePose2();
//                            break;
//                        case "PPG":
//                            indexer.OuttakePose2();
//                            break;
//                        default:
//                            indexer.OuttakePose1();
//                            break;
//                    }
//                }
                throwTimer.reset();
                stateThrow = 1;


                break;
            case 1:
//                    if (throwTimer.milliseconds() > 1200) {
                if (measuredRPM < (targetRPM + AUTO_THROW_RPM_TOLERANCE_HIGH) && measuredRPM > (targetRPM - AUTO_THROW_RPM_TOLERANCE_LOW) && throwTimer.milliseconds() > AUTO_THROW_RPM_WAIT_MS_EXTENDED) {
                    indexer.Push();
                    throwTimer.reset();
                    stateThrow = 2;
                }
                break;
            case 2:
                if (throwTimer.milliseconds() > THROW_PUSH_TO_DOWN_MS) {
                    indexer.Down();
                    throwTimer.reset();
                    stateThrow = 3;
                }
                break;
            // ----------------- A DOUA BILA -----------------
            case 3:
                if (throwTimer.milliseconds() > THROW_PUSH_TO_DOWN_MS) {
                    indexer.OuttakePose2();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose1();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose1();
//                                break;
//                            default:
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp2)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose1();
//                                break;
//                            default:
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp3)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose1();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose3();
//                                break;
//                            default:
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
                    throwTimer.reset();
                    stateThrow = 4;

                }
                break;
            case 4:
                if ((measuredRPM < (targetRPM + AUTO_THROW_RPM_TOLERANCE_HIGH)) && measuredRPM > (targetRPM - AUTO_THROW_RPM_TOLERANCE_LOW) && throwTimer.milliseconds() > AUTO_THROW_RPM_WAIT_MS_ALT) {
                    indexer.Push();
                    throwTimer.reset();
                    stateThrow = 5;
                }
                break;
            case 5:
                if (throwTimer.milliseconds() > THROW_PUSH_TO_DOWN_MS) {
                    indexer.Down();
                    throwTimer.reset();
                    stateThrow = 6;
                }
                break;
            // ----------------- A TREIA BILA -----------------
            case 6:
                if (throwTimer.milliseconds() > THROW_PUSH_TO_DOWN_MS) {
                    indexer.OuttakePose3();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose1();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose2();
//                                break;
//                            default:
//                                indexer.OuttakePose3();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp2)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose1();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose1();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose3();
//                                break;
//                            default:
//                                indexer.OuttakePose3();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp3)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose1();
//                                break;
//                            default:
//                                indexer.OuttakePose3();
//                                break;
//                        }
//                    }
                    throwTimer.reset();
                    stateThrow = 7;

                }
                break;
            case 7:
                if (measuredRPM < targetRPM + AUTO_THROW_RPM_TOLERANCE_HIGH && measuredRPM > targetRPM - AUTO_THROW_RPM_TOLERANCE_LOW && throwTimer.milliseconds() > AUTO_THROW_RPM_WAIT_MS_ALT) {
                    indexer.Push();
                    throwTimer.reset();
                    stateThrow = 8;
                }
                break;
            case 8:
                if (throwTimer.milliseconds() > THROW_PUSH_TO_DOWN_MS) {
                    indexer.Down();
                    stateThrow = 9;
                }
                break;
            case 9:
                if (throwTimer.milliseconds() > THROW_RESET_DELAY_MS) {
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
                if (distance <= COLLECT_DISTANCE_THRESHOLD_MM) {
                    indexer.PickPose2();
                    collecttimer.reset();
                    stateCollect = 1;

                }

                break;

            case 1:
                if (distance <= COLLECT_DISTANCE_THRESHOLD_MM && collecttimer.milliseconds() > COLLECT_DELAY_POSE1_TO_2_MS) {
                    indexer.PickPose3();
                    collecttimer.reset();
                    stateCollect = 2;
                }
                break;

            case 2: // PickPose3
                if (distance <= COLLECT_DISTANCE_THRESHOLD_MM && collecttimer.milliseconds() > COLLECT_DELAY_POSE2_TO_3_MS) {
                    collecttimer.reset();
                    targetRPM = targetRPM1;
                    indexer.OuttakePose1();
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
        outtake.Angle.setPosition(AUTO_OUTTAKE_ANGLE_DEFAULT);
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

        return (TURRET_POS_P * error)
                + (TURRET_POS_I * integralTurret)
                + (TURRET_POS_D * derivative)
                + (TURRET_POS_F * Math.signum(error));
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

