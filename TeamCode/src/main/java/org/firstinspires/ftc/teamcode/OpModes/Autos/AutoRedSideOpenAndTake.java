package org.firstinspires.ftc.teamcode.OpModes.Autos;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Limelight;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.WebcamTureta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/* DEPRECATED */
@Disabled
@Autonomous
public class AutoRedSideOpenAndTake extends OpMode {
    int stateThrow = 99;
    int stateCollect = 99;
    boolean ballsRemoved;
    ElapsedTime throwTimer;
    ElapsedTime collecttimer;

    //

    private double integral = 0;
    private double previousError = 0;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timerToSee = new ElapsedTime();
    double Time = 0;

    private int lastEncoder = 0;
    public static double targetRPM1 = AUTO_LAUNCHER_RPM_DEFAULT;
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
    boolean aruncareSelectiva = false;

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
    private final Pose startPose = new Pose(112, 134, Math.toRadians(90)); // Start Pose of our robot.

    public PathChain scorepreload;
    public PathChain firstPick;
    public PathChain scoreFirst;
    public PathChain pickScored;
    public PathChain scoreScored;
    public PathChain pickLast;
    public PathChain scoreLast;
    public void buildPaths() {

        scorepreload = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(112.000, 134.000),

                            new Pose(112, 160)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

            .build();

//        firstPick = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(94.000, 93.000),
//                                new Pose(70.000, 60.000),
//                                new Pose(125, 60)
//                        )
//                ).setTangentHeadingInterpolation()
//
//                .build();
//
//        scoreFirst = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(125, 60),
//                                new Pose(96.000, 67.000),
//                                new Pose(91.000, 89.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
//
//                .build();
//
//        pickScored = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(91.000, 89.000),
//                                new Pose(96.000, 48.000),
//                                new Pose(129.000, 61.000)
//                        )
//
//                ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(40))
//
//                .build();
//
//        scoreScored = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Pose(132.000, 59.000),
//                                new Pose(88.000, 66.000),
//                                new Pose(93.000, 84.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//
//                .build();
//
//        pickLast = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(93.000, 84.000),
//
//                                new Pose(119.000, 84.000)
//                        )
//                ).setTangentHeadingInterpolation()
//
//                .build();
//
//        scoreLast = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(126.000, 84.000),
//
//                                new Pose(84.000, 110.000)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(1);
                actionTimer.resetTimer();
//                targetPosition = -400;
//                tag = "GPP";
                break;
            case 1:
//                if (actionTimer.getElapsedTimeSeconds() > 15){
////                    targetRPM = 4800;
////                    indexer.KeepInside();
//                }
                if (actionTimer.getElapsedTimeSeconds() > 20) { // AUTO_PATH_TIMEOUT_SEC - path-specific
//                    stateThrow = 0;
                    setPathState(2);
                }
                break;
            case 2:
//                if (stateThrow == 99) {
                    follower.followPath(scorepreload);
//                    indexer.Stop();
//                    targetRPM = 0;
//                    targetPosition = 0;
//                    setPathState(3);
//                }
                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    stateThrow = 0;
//                    setPathState(2);
//                }
//                break;
//            case 2:
//
//                if (!follower.isBusy() && stateThrow == 99 ) {
//                    indexer.Colect();
//                    indexer.PickPose1();
//                    follower.followPath(firstPick);
//                    stateCollect = 0;
//                    targetRPM = targetRPM1;
//                    actionTimer.resetTimer();
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (!follower.isBusy() && (stateCollect == 99 || actionTimer.getElapsedTimeSeconds() > 2)){
//                    indexer.KeepInside();
//                    targetPosition = 550;
//                    follower.followPath(scoreFirst);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()){
//                    stateThrow = 0;
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//
//                if (stateThrow == 99) {
//                    follower.followPath(pickScored, 0.8 , true);
//                    indexer.Colect();
//                    stateCollect= 0;
//                    limelight.limelight.stop();
//                    targetRPM = targetRPM1;
//                    actionTimer.resetTimer();
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if (!follower.isBusy() && ( stateCollect == 99 || actionTimer.getElapsedTimeSeconds() > 7)){
//                    follower.followPath(scoreScored);
//                    targetPosition = 250;
//                    indexer.KeepInside();
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (!follower.isBusy()){
//                    stateThrow = 0;
//                    pathTimer.resetTimer();
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if (stateThrow == 99){
//                    stateCollect = 0;
//                    indexer.Colect();
//                    follower.followPath(pickLast);
//                    targetRPM = targetRPM1;
//                    setPathState( 9 );
//                    actionTimer.resetTimer();
//                }
//                break;
//            case 9:
//                if (!follower.isBusy() && (stateCollect == 99 || actionTimer.getElapsedTimeSeconds() > 2)){
//                    follower.followPath(scoreLast);
//                    targetPosition = 200;
//                    aruncareSelectiva = true;
//                    tag = "GPP";
//                    greenBallPickedAt = "PickPose3";
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if (!follower.isBusy()){
//                    stateThrow = 0;
//                    setPathState( 11);
//                }
//                break;
//            case 11:
//                if (stateThrow == 99){
//                    indexer.Stop();
//                    targetRPM = 0;
//                    targetPosition = 0;
//                    setPathState(12);
//                }
//                break;}
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
                if (!aruncareSelectiva){
                    indexer.OuttakePose1();
                }else {
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
                }
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
                    if (!aruncareSelectiva){
                        indexer.OuttakePose2();
                    }else {
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
                    }
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
                    if (!aruncareSelectiva){
                        indexer.OuttakePose3();
                    }else {
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
                    }
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
                    indexer.PickPose1();
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
                if (distance <= AUTO_COLLECT_DISTANCE_MM_TIGHT) {
                    indexer.PickPose2();
                    collecttimer.reset();
                    stateCollect = 1;

                }

                break;

            case 1:
                if (distance <= AUTO_COLLECT_DISTANCE_MM_TIGHT && collecttimer.milliseconds() > AUTO_COLLECT_DELAY_POSE1_TO_2_MS_FAST) {
                    indexer.PickPose3();
                    collecttimer.reset();
                    stateCollect = 2;
                }
                break;

            case 2: // PickPose3
                if (distance <= AUTO_COLLECT_DISTANCE_MM_TIGHT && collecttimer.milliseconds() > COLLECT_DELAY_POSE2_TO_3_MS) {
                    collecttimer.reset();
                    if (!aruncareSelectiva){
                        indexer.OuttakePose1();
                    }else {
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
