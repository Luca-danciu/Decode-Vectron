//package org.firstinspires.ftc.teamcode; // make sure this aligns with class location
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
//import org.firstinspires.ftc.teamcode.Hardware.Indexer;
//import org.firstinspires.ftc.teamcode.Hardware.Outtake;
//import org.firstinspires.ftc.teamcode.Hardware.WebcamTureta;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "AutoBlueTower")
//public class AutoBlueTower extends OpMode {
//
//    boolean detectionActive = true;
//    String detectedCase = "NONE";
//    public Indexer indexer = new Indexer();
//    public Outtake outtake = new Outtake();
//    public WebcamTureta webcam = new WebcamTureta();
//    public ColorSensorIndexer colorSensorIndexer = new ColorSensorIndexer();
//    public DistanceSensor distanceSensor;
//
//
//    String greenBallPickedAt = "Nicio bila verde preluata";
//    public String pp1 = "PickPose1";
//    public String pp2 = "PickPose2";
//    public String pp3 = "PickPose3";
//    int purplecount = 0;
//    int greencount = 0;
//    public double distance;
//    private Follower follower;
//    private Timer pathTimer, actionTimer, nextTask;
//    String tag;
//    private int pathState;
//    private final Pose startPose = new Pose(-35.35, -182, Math.toRadians(135));
//    private final Pose detectPose = new Pose(5, -145, Math.toRadians(175));
//    private final Pose scorePose = new Pose(5.5, -144.5  , Math.toRadians(220));
//    private final Pose pickup1PoseAngle = new Pose(7, -155, Math.toRadians(265));
//    private final Pose pickup1Pose= new Pose(6.5, -180, Math.toRadians(265));
//    private final Pose pickup2PoseAngle = new Pose(28, -155, Math.toRadians(265));
//    private final Pose pickup2Pose= new Pose(28, -190, Math.toRadians(265));
//    private final Pose pickup3PoseAngle = new Pose(50, -155, Math.toRadians(265));
//    private final Pose pickup3Pose = new Pose(50, -185, Math.toRadians(265));
//    private Path scorePreload;
//    private Path detectCase;
//    private Path grabPickup1Angle ;
//    private PathChain grabPickup1, scorePickup1, grabPickup2Angle, grabPickup2, scorePickup2, grabPickup3Angle, grabPickup3, scorePickup3;
//
//    public void buildPaths() {
//        detectCase = new Path(new BezierLine(startPose, detectPose));
//        detectCase.setLinearHeadingInterpolation(startPose.getHeading(), detectPose.getHeading());
//
//        scorePreload = new Path(new BezierLine(detectPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(detectPose.getHeading(), scorePose.getHeading());
//    scorePreload.setConstantInterpolation(startPose.getHeading()); */
//
//        grabPickup1Angle = new Path(new BezierLine(scorePose, pickup1PoseAngle));
//        grabPickup1Angle.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PoseAngle.getHeading());
//
//        grabPickup1 = new Path((new BezierCurve(pickup1PoseAngle, pickup1Pose)));
//        grabPickup1.setLinearHeadingInterpolation(pickup1PoseAngle.getHeading(), pickup1Pose.getHeading());
//        grabPickup1.setVelocityConstraint(10000);
//
//        grabPickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup1PoseAngle))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1PoseAngle.getHeading())
//                .setVelocityConstraint(1)
//                .addPath((new BezierCurve(pickup1PoseAngle, pickup1Pose)))
//                .setLinearHeadingInterpolation( pickup1PoseAngle.getHeading(), pickup1Pose.getHeading())
//                .build();
//
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose , scorePose))
//                .setLinearHeadingInterpolation( pickup1Pose.getHeading() ,scorePose.getHeading())
//                .build();
//        grabPickup2Angle = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose , pickup2PoseAngle))
//                .setLinearHeadingInterpolation(scorePose.getHeading() , pickup2PoseAngle.getHeading())
//                .build();
//        grabPickup2 = follower.pathBuilder()
//
//                .addPath(new BezierLine( pickup2PoseAngle ,pickup2Pose))
//                .setLinearHeadingInterpolation(pickup2PoseAngle.getHeading(), pickup2Pose.getHeading())
//                .build();
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose , scorePose))
//                .setLinearHeadingInterpolation( pickup2Pose.getHeading() ,scorePose.getHeading())
//                .build();
//        grabPickup3Angle = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose , pickup3PoseAngle))
//                .setLinearHeadingInterpolation(scorePose.getHeading() , pickup3PoseAngle.getHeading())
//                .setVelocityConstraint(1)
//                .build();
//        grabPickup3 = follower.pathBuilder()
//
//                .addPath(new BezierLine( pickup3PoseAngle ,pickup3Pose))
//                .setLinearHeadingInterpolation(pickup3PoseAngle.getHeading(), pickup3Pose.getHeading())
//                .build();
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup3Pose , scorePose))
//                .setLinearHeadingInterpolation( pickup3Pose.getHeading() ,scorePose.getHeading())
//                .build();
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(detectPose, pickup1Pose))
//                .setLinearHeadingInterpolation(detectPose.getHeading(), pickup1Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose, detectPose))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), detectPose.getHeading())
//                .build();
//
//        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(detectPose, pickup3Pose))
//                .setLinearHeadingInterpolation(detectPose.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup3Pose, detectPose))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), detectPose.getHeading())
//                .build();
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(detectCase);
//                outtake.Charge();
//                actionTimer.resetTimer();
//                setPathState(1);
//
//
//                break;
//            case 1:
//                if (actionTimer.getElapsedTimeSeconds() > 1){
//                tag = webcam.detectTag();
//                greenBallPickedAt = "PickPose1";
//                if (tag != null &&
//                        (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {
//
//                    detectedCase = tag;
//                    webcam.stopDetection();
//                    outtake.Charge();
//
//                    actionTimer.resetTimer();
//                    setPathState(2);
//
//                }}
//                break;
//            case 2:
//                if (!follower.isBusy() && tag != null) {
//                    follower.followPath(scorePreload, true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (actionTimer.getElapsedTimeSeconds() > 2) {
//                    indexer.KeepInside();
//                    outtake.AngleMax();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose3();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp2)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp3)) {
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
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    pathTimer.resetTimer();
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.Down();
//                    pathTimer.resetTimer();
//                    setPathState(6);
//                }
//            case 6:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.KeepInside();
//                    outtake.Charge();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
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
//                    pathTimer.resetTimer();
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4){
//                    indexer.Down();
//                    pathTimer.resetTimer();
//                    pathState = 9;
//                }
//            case 9:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.KeepInside();
//                    outtake.Charge();
//                    if (greenBallPickedAt.equals(pp1)) {
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
//                    pathTimer.resetTimer();
//                    pathState = 10;
//                }
//                break;
//            case 10:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    pathState = 11;
//                }
//                break;
//
//            case 11:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4){
//                    indexer.Down();
//                    outtake.StopLauncher();
//                    pathTimer.resetTimer();
//                    pathState = 12;
//                }
//                break;
//
//            case 12:
//                if (!follower.isBusy()){
//                    follower.followPath(grabPickup1Angle , true);
//                    pathTimer.resetTimer();
//                    pathState = 13;
//                }
//                break;
//
//            case 13:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.6){
//                    follower.followPath(grabPickup1 ,0.6, true);
//                    pathState = 14;
//                }
//                break;
//            case 14:
//                if ( pathTimer.getElapsedTimeSeconds() > 0.5 ) {
//                    nextTask.resetTimer();
//                    indexer.PickPose1();
//                    if (distance <= 60) {
//                        indexer.PickPose2();
//                        pathTimer.resetTimer();
//                        pathState = 15;
//                    }
//                }
//                break;
//
//            case 15:
//                if (pathTimer.getElapsedTimeSeconds() > 0.35){
//                    if (distance <=60 || nextTask.getElapsedTimeSeconds() > 1) {
//                        indexer.PickPose3();
//                        pathTimer.resetTimer();
//                        pathState = 16;
//                    }
//                }
//                break;
//            case 16:
//                if (pathTimer.getElapsedTimeSeconds() > 0.6 &&( distance <= 60 || nextTask.getElapsedTimeSeconds() > 2)){
//                    pathTimer.resetTimer();
//                    outtake.Charge();
//                    actionTimer.resetTimer();
//                    greenBallPickedAt = "PickPose3";
//                    pathState = 17;
//
//                }
//                break;
//            case 17:
//                follower.followPath(scorePickup1 , true);
//                actionTimer.resetTimer();
//                pathState = 18;
//                break;
//            case 18:
//                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    indexer.KeepInside();
//                    outtake.AngleMax();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose3();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp2)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp3)) {
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
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    pathTimer.resetTimer();
//                    setPathState(19);
//                }
//                break;
//            case 19:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    setPathState(20);
//                }
//                break;
//
//            case 20:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.Down();
//                    pathTimer.resetTimer();
//                    setPathState(21);
//                }
//            case 21:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.KeepInside();
//                    outtake.Charge();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
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
//                    pathTimer.resetTimer();
//                    setPathState(22);
//                }
//                break;
//            case 22:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    setPathState(23);
//                }
//                break;
//
//            case 23:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4){
//                    indexer.Down();
//                    pathTimer.resetTimer();
//                    pathState = 24;
//                }
//            case 24:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.KeepInside();
//                    outtake.Charge();
//                    if (greenBallPickedAt.equals(pp1)) {
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
//                    pathTimer.resetTimer();
//                    pathState = 25;
//                }
//                break;
//            case 25:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    pathState = 26;
//                }
//                break;
//
//            case 26:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4){
//                    indexer.Down();
//                    outtake.StopLauncher();
//                    pathTimer.resetTimer();
//                    pathState = 27;
//                }
//                break;
//            case 27:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.4){
//                    follower.followPath(grabPickup2Angle , true);
//                    pathState = 28;
//                }
//                break;
//            case 28:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.6){
//                    follower.followPath(grabPickup2, 0.6 , true);
//                    pathState = 29;
//                }
//                break;
//            case 29:
//                if ( pathTimer.getElapsedTimeSeconds() > 0.5 ) {
//                    nextTask.resetTimer();
//                    indexer.PickPose1();
//                    if (distance <= 60) {
//                        indexer.PickPose2();
//                        pathTimer.resetTimer();
//                        pathState = 30;
//                    }
//                }
//                break;
//
//            case 30:
//                if (pathTimer.getElapsedTimeSeconds() > 0.35){
//                    if (distance <=60) {
//                        indexer.PickPose3();
//                        pathTimer.resetTimer();
//                        pathState = 31;
//                    }
//                }
//                break;
//            case 31:
//                if (pathTimer.getElapsedTimeSeconds() > 0.6 && (distance <= 60 || nextTask.getElapsedTimeSeconds() > 2)){
//                    pathTimer.resetTimer();
//                    outtake.Charge();
//                    actionTimer.resetTimer();
//                    greenBallPickedAt = "PickPose2";
//                    pathState = 32;
//
//                }
//                break;
//            case 32:
//                follower.followPath(scorePickup2 , true);
//                actionTimer.resetTimer();
//                pathState = 33;
//                break;
//            case 33:
//                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    indexer.KeepInside();
//                    outtake.AngleMax();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose3();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp2)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp3)) {
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
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    pathTimer.resetTimer();
//                    setPathState(34);
//                }
//                break;
//            case 34:
//                if (pathTimer.getElapsedTimeSeconds() > 0.9) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    setPathState(35);
//                }
//                break;
//
//            case 35:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.Down();
//                    pathTimer.resetTimer();
//                    setPathState(36);
//                }
//            case 36:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.KeepInside();
//                    outtake.Charge();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
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
//                    pathTimer.resetTimer();
//                    setPathState(37);
//                }
//                break;
//            case 37:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    setPathState(38);
//                }
//                break;
//
//            case 38:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4){
//                    indexer.Down();
//                    pathTimer.resetTimer();
//                    pathState = 39;
//                }
//            case 39:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.KeepInside();
//                    outtake.Charge();
//                    if (greenBallPickedAt.equals(pp1)) {
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
//                    pathTimer.resetTimer();
//                    pathState = 40;
//                }
//                break;
//            case 40:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    pathState = 41;
//                }
//                break;
//
//            case 41:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4){
//                    indexer.Down();
//                    outtake.StopLauncher();
//                    pathTimer.resetTimer();
//                    pathState = 42;
//                }
//                break;
//            case 42:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.4){
//                    follower.followPath(grabPickup3Angle , true);
//                    pathState = 43;
//                }
//                break;
//            case 43:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.6){
//                    follower.followPath(grabPickup3, 0.6 , true);
//                    pathState = 44;
//                }
//                break;
//            case 44:
//                if ( pathTimer.getElapsedTimeSeconds() > 0.5 ) {
//                    indexer.PickPose1();
//                    if (distance <= 60) {
//                        indexer.PickPose2();
//                        pathTimer.resetTimer();
//                        pathState = 45;
//                    }
//                }
//                break;
//
//            case 45:
//                if (pathTimer.getElapsedTimeSeconds() > 0.35){
//                    if (distance <=60) {
//                        indexer.PickPose3();
//                        pathTimer.resetTimer();
//                        pathState = 46;
//                    }
//                }
//                break;
//            case 46:
//                if (pathTimer.getElapsedTimeSeconds() > 0.6 && distance <= 60){
//                    pathTimer.resetTimer();
//                    outtake.Charge();
//                    actionTimer.resetTimer();
//                    greenBallPickedAt = "PickPose1";
//                    pathState = 47;
//
//                }
//                break;
//            case 47:
//                follower.followPath(scorePickup2 , true);
//                actionTimer.resetTimer();
//                pathState = 48;
//                break;
//            case 48:
//                if (actionTimer.getElapsedTimeSeconds() > 1.5) {
//                    indexer.KeepInside();
//                    outtake.AngleMax();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose3();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp2)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
//                                break;
//                            case "PGP":
//                                indexer.OuttakePose2();
//                                break;
//                            case "PPG":
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    if (greenBallPickedAt.equals(pp3)) {
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
//                                indexer.OuttakePose2();
//                                break;
//                        }
//                    }
//                    pathTimer.resetTimer();
//                    setPathState(49);
//                }
//                break;
//            case 49:
//                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    setPathState(50);
//                }
//                break;
//
//            case 50:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.Down();
//                    pathTimer.resetTimer();
//                    setPathState(51);
//                }
//            case 51:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.KeepInside();
//                    outtake.Charge();
//                    if (greenBallPickedAt.equals(pp1)) {
//                        String caseUsed = detectedCase;
//
//                        switch (caseUsed) {
//                            case "GPP":
//                                indexer.OuttakePose3();
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
//                    pathTimer.resetTimer();
//                    setPathState(52);
//                }
//                break;
//            case 52:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    setPathState(53);
//                }
//                break;
//
//            case 53:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4){
//                    indexer.Down();
//                    pathTimer.resetTimer();
//                    pathState = 54;
//                }
//            case 54:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    indexer.KeepInside();
//                    outtake.Charge();
//                    if (greenBallPickedAt.equals(pp1)) {
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
//                    pathTimer.resetTimer();
//                    pathState = 55;
//                }
//                break;
//            case 55:
//                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    indexer.Push();
//                    pathTimer.resetTimer();
//                    pathState = 56;
//                }
//                break;
//
//            case 56:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4){
//                    indexer.Down();
//                    outtake.StopLauncher();
//                    pathTimer.resetTimer();
//                    pathState = 57;
//                }
//                break;
//
//        }
//    }
//
//    /**
//     * These change the states of the paths and actions. It will also reset the timers of the individual switches
//     **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    /**
//     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
//     **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot, these must be called continuously in order to work
//        distance = distanceSensor.getDistance(DistanceUnit.MM);
//        autonomousPathUpdate();
//        telemetry.addData("path state", pathState);
//
//        follower.update();
//        autonomousPathUpdate();
//
//        // Feedback to Driver Hub for debugging
//        telemetry.addData("Caz" , tag);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//    /**
//     * This method is called once at the init of the OpMode.
//     **/
//    @Override
//    public void init() {
//
//        indexer.indexerinit(hardwareMap);
//        outtake.outtakeinit(hardwareMap);
//        webcam.initwebcam(hardwareMap);
//        colorSensorIndexer.initcolorsensor(hardwareMap);
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");
//        indexer.PickPose1();
//        outtake.AngleMin();
//
//        // Timere
//        pathTimer = new Timer();
//        actionTimer = new Timer();
//        nextTask  = new Timer();
//
//        telemetry.addLine("Init complete");
//        telemetry.update();
//
//
//        follower = Constants.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//
//    }
//
//    /**
//     * This method is called continuously after Init while waiting for "play".
//     **/
//    @Override
//    public void init_loop() {
//        if (detectionActive) {
//            webcam.startDetection();
//            String tag = webcam.detectTag();
//            if (tag != null &&
//                    (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {
//
//                detectedCase = tag;   // Salvăm cazul
//            }
//        }
//
//        telemetry.addData("Caz detectat", detectedCase);
//        telemetry.update();
//    }
//
//    /**
//     * This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system
//     **/
//    @Override
//    public void start() {
//
//        setPathState(0);
//        indexer.PickPose1();
//        webcam.startDetection();
//        detectionActive = true;
//
//        pathState = 0;
//    }
//
//    /**
//     * We do not use this because everything should automatically disable
//     **/
//    @Override
//    public void stop() {
//    }
//}

//public static class Paths {
//
//    public PathChain pickup1Prepare;
//    public PathChain pickup1;
//    public PathChain scorePickup1;
//    public PathChain pickup2Prepare;
//    public PathChain pickup2;
//    public PathChain scorePickup2;
//    public PathChain pickup3Prepare;
//    public PathChain pickup3;
//    public PathChain scorePickup3;
//    public PathChain park;
//
//    public Paths(Follower follower) {
//        pickup1Prepare = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(63.000, 9.000), new Pose(44.000, 36.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
//                .build();
//
//        pickup1 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(44.000, 36.000), new Pose(10.000, 36.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                .build();
//
//        scorePickup1 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(10.000, 36.000), new Pose(60.000, 19.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
//                .build();
//
//        pickup2Prepare = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(60.000, 19.000), new Pose(44.000, 60.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
//                .build();
//
//        pickup2 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(44.000, 60.000), new Pose(10.000, 60.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                .build();
//
//        scorePickup2 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(10.000, 60.000), new Pose(60.000, 19.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
//                .build();
//
//        pickup3Prepare = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(60.000, 19.000), new Pose(14.500, 18.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(200))
//                .build();
//
//        pickup3 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(14.500, 18.000), new Pose(14.500, 11.400))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(-130))
//                .build();
//
//        scorePickup3 = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(14.500, 11.400), new Pose(60.000, 19.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(120))
//                .build();
//
//        park = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(60.000, 19.000), new Pose(40.000, 19.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
//                .build();
//    }
//}
//


import com.pedropathing.geometry.Pose;

