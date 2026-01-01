package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.WebcamTureta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoBlueSideTower extends OpMode {
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

    public DcMotor launcher;

    boolean detectionActive = true;
    String detectedCase = "NONE";
    public Indexer indexer = new Indexer();
    public Outtake outtake = new Outtake();

    public WebcamTureta webcam = new WebcamTureta();
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
    private final Pose startPose = new Pose(63, 135, Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose scorePosePreload = new Pose(55, 84, Math.toRadians(180));
    private final Pose scorePose123 = new Pose(55, 84, Math.toRadians(225));
    private final Pose pickup1Pose = new Pose(17, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2PosePrepare = new Pose(42, 60, Math.toRadians(180));// Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(15, 60, Math.toRadians(180));// Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3PosePrepare = new Pose(42,36, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(15 , 36, Math.toRadians(180));
    private final Pose park = new Pose(35 , 84, Math.toRadians(180));
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2Prepare , grabPickup2 , scorePickup2, grabPickup3Prepare,grabPickup3, scorePickup3 , parkare;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePosePreload));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePosePreload.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePosePreload, pickup1Pose))
                .setLinearHeadingInterpolation(scorePosePreload.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose123))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose123.getHeading())
                .build();

        grabPickup2Prepare = follower.pathBuilder()
                .addPath(new BezierLine(scorePose123, pickup2PosePrepare))
                .setLinearHeadingInterpolation(scorePose123.getHeading(), pickup2PosePrepare.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2PosePrepare, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2PosePrepare.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose123))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose123.getHeading())
                .build();

        grabPickup3Prepare = follower.pathBuilder()
                .addPath(new BezierLine(scorePose123, pickup3PosePrepare))
                .setLinearHeadingInterpolation(scorePose123.getHeading(), pickup3PosePrepare.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3PosePrepare, pickup3Pose))
                .setLinearHeadingInterpolation(pickup3PosePrepare.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose123))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose123.getHeading())
                .build();

        parkare = follower.pathBuilder()
                .addPath(new BezierLine(scorePose123, park))
                .setLinearHeadingInterpolation(scorePose123.getHeading(), park.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                targetRPM1 = 5000;
                actionTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if (actionTimer.getElapsedTimeSeconds() > 0.3){
                    tag = webcam.detectTag();
                    greenBallPickedAt = "PickPose1";
                    if (tag != null &&
                            (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {

                        detectedCase = tag;
                        webcam.stopDetection();
                        outtake.Charge();
                        targetPosition = 400;
                        actionTimer.resetTimer();
                        setPathState(2);
                    }
                    if (!follower.isBusy() && tag == null){
                        targetPosition = 800;
                    }
                }
                break;

            case 2:
                if (!follower.isBusy() && tag != null){
                    stateThrow = 0;
                    setPathState(3);
                    targetPosition = 400;
                }
                break;

            case 3:
                if (stateThrow == 99) {
                    follower.followPath(grabPickup1 , 0.6 , true);
                    indexer.Colect();
                    greenBallPickedAt = "PickPose3";
                    pathTimer.resetTimer();
                    stateCollect = 0;
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && (stateCollect == 99 || pathTimer.getElapsedTimeSeconds() > 1.5)){
                    follower.followPath(scorePickup1);
                    targetPosition = 800;
                    targetRPM = 5000;
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    stateThrow = 0;
                    setPathState(6);
                }
                break;
            case 6:
                if (stateThrow == 99){
                    follower.followPath(grabPickup2Prepare , true);
                    greenBallPickedAt = "PickPose2";
                    pathTimer.resetTimer();
                    indexer.Colect();
                    stateCollect = 0;
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.followPath(grabPickup2 , 0.6 , true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() && (stateCollect == 99 || pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    follower.followPath(scorePickup2);
                    targetPosition = 800;
                    targetRPM = 5000;
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()){
                    stateThrow = 0;
                    setPathState(10);
                }
                break;
            case 10:
                if (stateThrow == 99){
                    follower.followPath(grabPickup3Prepare , true);
                    greenBallPickedAt = "PickPose1";
                    pathTimer.resetTimer();
                    indexer.Colect();
                    stateCollect = 0;
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()){
                    follower.followPath(grabPickup3 , 0.6 , true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() && (stateCollect == 99 || pathTimer.getElapsedTimeSeconds() > 1.5)) {
                    follower.followPath(scorePickup3);
                    targetPosition = 800;
                    targetRPM = 5000;
                    setPathState(13);
                }
                break;
            case 13:
                if (stateThrow == 99){
                    follower.followPath(parkare);
                    setPathState(14);
                }


//            case 1:
//                if(!follower.isBusy()) {
//                    follower.followPath(grabPickup1,0.6 ,true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if(!follower.isBusy()) {
//                    follower.followPath(scorePickup1,true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if(!follower.isBusy()) {
//                    follower.followPath(grabPickup2Prepare);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if(!follower.isBusy()) {
//                    follower.followPath(grabPickup2 , 0.6 , true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if(!follower.isBusy()) {
//                    follower.followPath(scorePickup2);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if(!follower.isBusy()) {
//                    follower.followPath(grabPickup3Prepare ,true );
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if(!follower.isBusy()) {
//                    follower.followPath(grabPickup3 , 0.6 , true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if(!follower.isBusy()) {
//                    follower.followPath(scorePickup3);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if(!follower.isBusy()) {
//                    follower.followPath(parkare);
//                    setPathState(-1);
//                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

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
        distance = distanceSensor.getDistance(DistanceUnit.MM);
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);

        switch (stateThrow) {
            // ----------------- PRIMA BILA -----------------
            case 0:
                    timerToSee.reset();
                    targetRPM = targetRPM1;
                    if (targetRPM1 == 6000) {
                        pidTargetRPM = 12000;
                    } else {
                        pidTargetRPM = targetRPM1;
                    }

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
                if (pidTargetRPM == 12000) {
                    if (measuredRPM > (targetRPM + 900)) {
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
                    indexer.Down();
                    pidTargetRPM = 0;
                    stateThrow = 99;
                    Time = timerToSee.seconds();
                    greenBallPickedAt = "Nicio bila verde preluata";
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

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Caz" , tag);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
        webcam.initwebcam(hardwareMap);
        colorSensorIndexer.initcolorsensor(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");
        indexer.PickPose1();
        outtake.AngleMin();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        nextTask  = new Timer();



        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

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
}
