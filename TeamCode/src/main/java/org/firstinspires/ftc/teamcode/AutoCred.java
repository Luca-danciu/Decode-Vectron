package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensorIndexer;
import org.firstinspires.ftc.teamcode.Hardware.IMUIndexer;
import org.firstinspires.ftc.teamcode.Hardware.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.WebcamTureta;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import kotlin.properties.ObservableProperty;

@Autonomous(name = "Autocred")

public class AutoCred extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
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

    ColorSensorIndexer.DetectedColor detectedColor1, detectedColor2, detectedColor3;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                String tag = webcam.detectTag();
                greenBallPickedAt = "PickPose1";
                if (tag != null &&
                        (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {

                    detectedCase = tag;
                    webcam.stopDetection();
                    outtake.Charge();

                    actionTimer.resetTimer();
                    pathState = 1;
                }
                break;
            case 1:
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    indexer.KeepInside();
                    outtake.AngleMax();
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
                        }
                    }
                    pathTimer.resetTimer();
                    pathState = 2;
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 3;
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.Down();
                    pathTimer.resetTimer();
                    pathState = 4;
                }
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.KeepInside();
                    outtake.Charge();
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
                    pathTimer.resetTimer();
                    pathState = 5;
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 6;
                }
                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    indexer.Down();
                    pathTimer.resetTimer();
                    pathState = 7;
                }
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.KeepInside();
                    outtake.Charge();
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
                    pathTimer.resetTimer();
                    pathState = 8;
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 9;
                }
                break;

            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    indexer.Down();
                    outtake.StopLauncher();
                    pathTimer.resetTimer();
                    pathState = 10;
                }
                break;

            case 10:
                if ( pathTimer.getElapsedTimeSeconds() > 0.5 ) {
                    indexer.PickPose1();
                    if (distance <= 60) {
                        indexer.PickPose2();
                        pathTimer.resetTimer();
                        pathState = 11;
                    }
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 0.6){
                    if (distance <=60) {
                        indexer.PickPose3();
                        pathTimer.resetTimer();
                        pathState = 12;
                    }
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.6 && distance <= 60){
                    pathTimer.resetTimer();
                    outtake.Charge();
                    greenBallPickedAt = "PickPose3";
                    pathState = 13;

                }
                break;

            case 13:
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    indexer.KeepInside();
                    outtake.AngleMax();
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
                        }
                    }
                    pathTimer.resetTimer();
                    pathState = 14;
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 15;
                }
                break;

            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.Down();
                    pathTimer.resetTimer();
                    pathState = 16;
                }
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.KeepInside();
                    outtake.Charge();
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
                    pathTimer.resetTimer();
                    pathState = 17;
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 18;
                }
                break;

            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    indexer.Down();
                    pathTimer.resetTimer();
                    pathState = 19;
                }
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.KeepInside();
                    outtake.Charge();
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
                    pathTimer.resetTimer();
                    pathState = 20;
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 21;
                }
                break;

            case 21:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    indexer.Down();
                    outtake.StopLauncher();
                    pathTimer.resetTimer();
                    pathState = 22;
                }
                break;
            case 22:
                if ( pathTimer.getElapsedTimeSeconds() > 0.5 ) {
                    indexer.PickPose1();
                    if (distance <= 60) {
                        indexer.PickPose2();
                        pathTimer.resetTimer();
                        pathState = 23;
                    }
                }
                break;

            case 23:
                if (pathTimer.getElapsedTimeSeconds() > 0.6){
                    if (distance <=60) {
                        indexer.PickPose3();
                        pathTimer.resetTimer();
                        pathState = 24;
                    }
                }
                break;
            case 24:
                if (pathTimer.getElapsedTimeSeconds() > 0.6 && distance <= 60){
                    pathTimer.resetTimer();
                    outtake.Charge();
                    greenBallPickedAt = "PickPose2";
                    pathState = 25;

                }
                break;

            case 25:
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    indexer.KeepInside();
                    outtake.AngleMax();
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
                        }
                    }
                    pathTimer.resetTimer();
                    pathState = 26;
                }
                break;
            case 26:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 27;
                }
                break;

            case 27:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.Down();
                    pathTimer.resetTimer();
                    pathState = 28;
                }
            case 28:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.KeepInside();
                    outtake.Charge();
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
                    pathTimer.resetTimer();
                    pathState = 29;
                }
                break;
            case 29:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 30;
                }
                break;

            case 30:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    indexer.Down();
                    pathTimer.resetTimer();
                    pathState = 31;
                }
            case 31:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.KeepInside();
                    outtake.Charge();
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
                    pathTimer.resetTimer();
                    pathState = 32;
                }
                break;
            case 32:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 33;
                }
                break;

            case 33:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    indexer.Down();
                    outtake.StopLauncher();
                    pathTimer.resetTimer();
                    pathState = 34;
                }
                break;
            case 34:
                if ( pathTimer.getElapsedTimeSeconds() > 0.5 ) {
                    indexer.PickPose1();
                    if (distance <= 60) {
                        indexer.PickPose2();
                        pathTimer.resetTimer();
                        pathState = 35;
                    }
                }
                break;

            case 35:
                if (pathTimer.getElapsedTimeSeconds() > 0.6){
                    if (distance <=60) {
                        indexer.PickPose3();
                        pathTimer.resetTimer();
                        pathState = 36;
                    }
                }
                break;
            case 36:
                if (pathTimer.getElapsedTimeSeconds() > 0.6 && distance <= 60){
                    pathTimer.resetTimer();
                    outtake.Charge();
                    greenBallPickedAt = "PickPose1";
                    pathState = 37;

                }
                break;

            case 37:
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    indexer.KeepInside();
                    outtake.AngleMax();
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
                        }
                    }
                    pathTimer.resetTimer();
                    pathState = 38;
                }
                break;
            case 38:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 39;
                }
                break;

            case 39:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.Down();
                    pathTimer.resetTimer();
                    pathState = 40;
                }
            case 40:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.KeepInside();
                    outtake.Charge();
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
                    pathTimer.resetTimer();
                    pathState = 41;
                }
                break;
            case 41:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 42;
                }
                break;

            case 42:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    indexer.Down();
                    pathTimer.resetTimer();
                    pathState = 43;
                }
            case 43:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    indexer.KeepInside();
                    outtake.Charge();
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
                    pathTimer.resetTimer();
                    pathState = 44;
                }
                break;
            case 44:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    indexer.Push();
                    pathTimer.resetTimer();
                    pathState = 45;
                }
                break;

            case 45:
                if (pathTimer.getElapsedTimeSeconds() > 0.4){
                    indexer.Down();
                    outtake.StopLauncher();
                    pathTimer.resetTimer();
                    pathState = 46;
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
        distance = distanceSensor.getDistance(DistanceUnit.MM);
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        indexer.indexerinit(hardwareMap);
        outtake.outtakeinit(hardwareMap);
        webcam.initwebcam(hardwareMap);
        colorSensorIndexer.initcolorsensor(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");


        // Timere
        pathTimer = new Timer();
        actionTimer = new Timer();

        telemetry.addLine("Init complete");
        telemetry.update();

    }

    @Override
    public void init_loop() {
        if (detectionActive) {
            webcam.startDetection();
            String tag = webcam.detectTag();
            if (tag != null &&
                    (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {

                detectedCase = tag;   // Salvăm cazul
            }
        }

        telemetry.addData("Caz detectat", detectedCase);
        telemetry.update();

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        indexer.PickPose1();
        webcam.startDetection();
        detectionActive = true;

        pathState = 0;

    }
    @Override
    public void stop() {
    }

}





