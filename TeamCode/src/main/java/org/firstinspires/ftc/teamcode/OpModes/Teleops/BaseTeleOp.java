package org.firstinspires.ftc.teamcode.OpModes.Teleops;

import static org.firstinspires.ftc.teamcode.OpmodesConstants.*;

import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

/**
 * Base TeleOp class used by all alliance variants (Red, Blue, BNG).
 * <ul>
 *   <li>Odometry via pedroPathing Follower + AprilTag correction</li>
 *   <li>Turret: tower aim / Limelight tracking</li>
 *   <li>Velocity-based launcher, 3-ball throw, 5-state collect</li>
 *   <li>Intake, ascent, LEDs</li>
 * </ul>
 * Subclasses provide alliance-specific poses, tower positions, and AprilTag ID.
 */
public abstract class BaseTeleOp extends LinearOpMode {

    protected Drivetrain drivetrain = new Drivetrain();
    protected Indexer indexer = new Indexer();
    protected Outtake outtake = new Outtake();
    protected IMUIndexer imuIndexer = new IMUIndexer();
    protected ColorSensorIndexer colorSensorIndexer = new ColorSensorIndexer();
    protected Limelight limelight = new Limelight();
    protected Ascent ascent = new Ascent();

    protected Follower follower;
    private Supplier<PathChain> parcare;
    FieldManager panelsField = PanelsField.INSTANCE.getField();

    protected boolean pathFinished = false;
    protected boolean automatedDrive = false;

    protected DistanceSensor distanceSensor;
    protected DcMotorEx launcher;
    protected DcMotorEx turret;
    protected DigitalChannel ledRedD, ledGreenD, ledRedS, ledGreenS;

    protected static final int MAX_TICKST = (int) (180 * TURRET_TICKS_PER_DEGREE);
    protected static final int MIN_TICKST = -MAX_TICKST;
    protected static final int MAX_TICKS = (int) (180 * TURRET_TICKS_PER_DEGREE_LIMELIGHT);
    protected static final int MIN_TICKS = -MAX_TICKS;
    protected int turretHoldTicks = 0;
    protected boolean resumeFromLast = false;
    protected boolean turetaToZero = false;
    private double integralTurret = 0;
    private double lastErrorTurret = 0;
    private double lastErrorT = 0;
    protected static int targetPosition = 0;

    protected double distance = 0;
    protected int purplecount = 0;
    protected int greencount = 0;
    protected double timeThrow = 0;
    protected boolean takeOUT = false;
    protected boolean allBallsIn = false;
    protected boolean stopIntake = false;
    protected boolean ballsRemoved = false;

    protected final ElapsedTime timerToSee = new ElapsedTime();
    protected final ElapsedTime takeGreenBallOut = new ElapsedTime();
    protected final ElapsedTime takePurpleBallOut = new ElapsedTime();
    protected boolean ridic = false;
    protected int lastCollectState = 0;

    // --- BNG config (overridable by subclasses) ---
    protected int towerX;
    protected int towerY;
    protected double servo1 = 0.7;
    protected double servo2 = 0.5;
    protected double servo3 = 0.25;
    protected double servo4 = 0.0;
    protected double targetVel1 = 1250;
    protected double targetVel2 = 1100;
    protected double targetVel3 = 900;
    protected double targetVelocity = 0;
    protected double s1 = 0, s2 = 0, s3 = 0;
    protected double timerClose = 300;
    protected double timerFar1 = 800;
    protected double timerFar2 = 1200;
    protected boolean FAR = false;
    protected boolean CLOSE = false;
    protected static double launcherP = 100, launcherI = 0, launcherD = 0, launcherF = 100;

    // ==================== Abstract ====================

    protected abstract Pose getStartPose();

    protected abstract Pose getParkPose();

    protected abstract int getParkHeadingDeg();

    protected abstract int getTowerX();

    protected abstract int getTowerY();

    protected abstract Pose getTurretOverridePose();

    protected abstract int getAprilTagIdForTurret();

    /**
     * Optional: override pose when circle pressed. Default null = no override.
     */
    protected Pose getCircleOverridePose() { return null; }

    /**
     * Override to update towerX/towerY from distance (e.g. BNG FAR mode). Default no-op.
     */
    protected void updateTowerFromDistance(double distInches) {}

    // ==================== Init ====================

    protected void initBaseTeleOp(HardwareMap hardwareMap) {
        colorSensorIndexer.initcolorsensor(hardwareMap);
        indexer.indexerinit(hardwareMap);
        outtake.outtakeinit(hardwareMap);
        imuIndexer.init(hardwareMap);
        limelight.limelightinit(hardwareMap);
        drivetrain.drivetraininit(hardwareMap);
        ascent.ascentinit(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(getStartPose());

        // Panels Init
        panelsField.setOffsets(FieldPresets.INSTANCE.getPEDRO_PATHING());

        parcare = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, getParkPose())))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, Math.toRadians(getParkHeadingDeg()), PARK_HEADING_BLEND))
                .build();
    }

    protected void initTeleopHardware(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "SenzorIntakeCH");
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        turret = hardwareMap.get(DcMotorEx.class, "Tureta");

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
        initLauncher();
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    protected void initLauncher() {
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // ==================== Localization & Drive ====================

    /** Odometry pose (wheel encoder) and Limelight pose (AprilTag), populated each updateLocalization(). */
    protected Pose lastOdometryPose;
    protected Pose lastLimelightPose;

    protected void updateLocalization() {
        follower.update();
        lastOdometryPose = follower.getPose();
        lastLimelightPose = limelight.getFieldPoseFromAprilTag(follower.getPose());
        // if (lastLimelightPose != null) {
        //     follower.setPose(lastLimelightPose);
        // }
    }

    protected void handleDriveAndPark() {
        if (gamepad1.cross && !automatedDrive) {
            follower.followPath(parcare.get());
            automatedDrive = true;
            pathFinished = false;
        }

        if (!automatedDrive) {
            if (Math.abs(gamepad1.left_stick_y) < DRIVE_STICK_DEADZONE
                    && Math.abs(gamepad1.left_stick_x) < DRIVE_STICK_DEADZONE
                    && Math.abs(gamepad1.right_stick_x) < DRIVE_STICK_DEADZONE) {
                drivetrain.activeBrake();
            } else {
                drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
        }

        if (automatedDrive && !follower.isBusy() && !pathFinished) {
            pathFinished = true;
            drivetrain.drive(0, 0, 0);
            drivetrain.activeBrake();
        }

        if (pathFinished && gamepad1.square) {
            automatedDrive = false;
            pathFinished = false;
        }
    }

    // ==================== Main loop ====================

    public void runOpMode() throws InterruptedException {
        int stateThrow = 0;
        int stateCollect = 0;
        int stateTureta = 0;
        ElapsedTime statetureta = new ElapsedTime();
        ElapsedTime throwTimer = new ElapsedTime();
        ElapsedTime collecttimer = new ElapsedTime();

        initBaseTeleOp(hardwareMap);
        initTeleopHardware(hardwareMap);
        towerX = getTowerX();
        towerY = getTowerY();
        timerToSee.reset();

        waitForStart();

        indexer.PickPose1();
        indexer.Down();
        limelight.limelight.start();
        statetureta.reset();
        ascent.Prins();

        while (opModeIsActive() && !isStopRequested()) {
            launcher.setVelocityPIDFCoefficients(launcherP, launcherI, launcherD, launcherF);
            launcher.setVelocity(targetVelocity);

            updateLocalization();



            PanelsTelemetry.INSTANCE.getTelemetry().addData("last odo pose", lastOdometryPose != null ? lastOdometryPose : "");
            PanelsTelemetry.INSTANCE.getTelemetry().addData("last lime pose", lastLimelightPose != null ? lastLimelightPose : "");
            PanelsTelemetry.INSTANCE.getTelemetry().addData("lime debug", limelight.poseDebugReason + " tags=" + limelight.poseDebugTagsSeen);
            PanelsTelemetry.INSTANCE.getTelemetry().update();


            // Draw circles: blue = odometry pose, red = Limelight/AprilTag pose
            panelsField.setStyle(new Style("#3F51B5", "#3F51B5", 0.75));
            panelsField.moveCursor(follower.getPose().getX(), follower.getPose().getY());
            panelsField.circle(10);
            if (lastLimelightPose != null) {
                panelsField.setStyle(new Style("#E53935", "#E53935", 0.75));
                panelsField.moveCursor(lastLimelightPose.getX(), lastLimelightPose.getY());
                panelsField.circle(10);
            }
            panelsField.update();

            Pose circlePose = getCircleOverridePose();
            if (circlePose != null && gamepad1.circle) {
                follower.setPose(circlePose);
            }

            handleDriveAndPark();

            distance = distanceSensor.getDistance(DistanceUnit.MM);
            updateLeds();

            if (gamepad1.left_bumper) {
                follower.setPose(getTurretOverridePose());
            }
            stateTureta = runTurretStateMachine(stateTureta, statetureta);

            if (gamepad1.dpad_up) {
                ascent.Eliberat();
                ridic = true;
            }
            if (gamepad1.dpad_left) {
                ascent.Prins();
            }
            if (ridic) {
                targetPosition = 0;
                indexer.Stop();
                drivetrain.activeBrake();
            }
            if (gamepad1.dpad_down) {
                ascent.Ridicare(ASCENT_POWER_D, ASCENT_POWER_S);
            } else {
                ascent.Ridicare(0, 0);
            }

            limelight.limelight.start();

            double dx = towerX - follower.getPose().getX();
            double dy = towerY - follower.getPose().getY();
            double dist = Math.sqrt(dx * dx + dy * dy);

            updateTowerFromDistance(dist);

            if (dist < 45) {
                targetVelocity = targetVel3;
                CLOSE = true;
                FAR = false;
            } else if (dist > 45 && dist < 105) {
                targetVelocity = targetVel2;
                CLOSE = false;
                FAR = false;
            } else if (dist > 105) {
                targetVelocity = targetVel1;
                CLOSE = false;
                FAR = true;
            }

            if (dist < 45) {
                s1 = 0.7;
                s2 = 0.7;
                s3 = 0.7;
            } else if (dist > 45 && dist < 50) {
                s1 = 0.1;
                s2 = 0.25;
                s3 = 0.35;
            } else if (dist > 50 && dist < 60) {
                s1 = 0;
                s2 = 0.25;
                s3 = 0.4;
            } else if (dist > 60 && dist < 70) {
                s1 = 0;
                s2 = 0.25;
                s3 = 0.43;
            } else if (dist > 70 && dist < 80) {
                s1 = 0.1;
                s2 = 0.25;
                s3 = 0.45;
            } else if (dist > 80 && dist < 90) {
                s1 = 0.05;
                s2 = 0.3;
                s3 = 0.45;
            } else if (dist > 90) {
                s1 = 0;
                s2 = 0.3;
                s3 = 0.3;
            }

            if (dist < 45 && stateThrow == 0) {
                outtake.Angle.setPosition(servo1);
            } else if (dist > 45 && stateThrow == 0) {
                outtake.Angle.setPosition(servo4);
            }

            stateThrow = runThrowStateMachine(stateThrow, throwTimer);
            beforeCollectCycle();
            stateCollect = runCollectStateMachine(stateCollect, collecttimer);
            lastCollectState = stateCollect;

            handleIntake(collecttimer);
            if (gamepad1.options) {
                stateCollect = 0;
                takeOUT = false;
                stopIntake = false;
            }

            addPositionTelemetry();
            telemetry.addData("Stare camera", limelight.limelight.isRunning() ? "Pornita" : "Oprita");
            telemetry.addData("Distanta", dist);
            telemetry.addData("statethrow", stateThrow);
            telemetry.addData("Target RPM", targetVelocity);
            telemetry.addData("Measured RPM", launcher.getVelocity());
            telemetry.addData("TimeThrow", timeThrow);
            telemetry.addData("Servo", outtake.Angle.getPosition());
            telemetry.update();
        }
        limelight.limelight.stop();
    }

    protected void updateLeds() {
        if (allBallsIn) {
            ledRedD.setState(true);
            ledGreenD.setState(false);
            ledRedS.setState(true);
            ledGreenS.setState(false);
        } else {
            ledRedD.setState(false);
            ledGreenD.setState(true);
            ledRedS.setState(false);
            ledGreenS.setState(true);
        }
    }

    protected int runTurretStateMachine(int stateTureta, ElapsedTime statetureta) {
        switch (stateTureta) {
            case 0:
                int targetTicks;
                if (resumeFromLast) {
                    targetTicks = turretHoldTicks;
                    resumeFromLast = false;
                } else {
                    double dx = towerX - follower.getPose().getX();
                    double dy = towerY - follower.getPose().getY();
                    double alpha1 = Math.toDegrees(Math.atan2(dy, dx));
                    double headingRobot = Math.toDegrees(follower.getHeading());
                    double headingTuretaGrade = alpha1 - headingRobot;
                    if (headingTuretaGrade > 180) headingTuretaGrade -= 360;
                    if (headingTuretaGrade < -180) headingTuretaGrade += 360;
                    targetTicks = (int) (headingTuretaGrade * TURRET_TICKS_PER_DEGREE);
                }
                int currentTicks = turret.getCurrentPosition();
                int error = targetTicks - currentTicks;
                double powerT = TURRET_KP * error + TURRET_KD * (error - lastErrorT);
                lastErrorT = error;
                powerT = Math.max(-1, Math.min(1, powerT));
                //TODO: revert
//                turret.setPower(powerT);
                if (gamepad1.share && statetureta.milliseconds() > TURRET_STATE_DEBOUNCE_MS) {
                    statetureta.reset();
                    return 1;
                }
                break;
            case 1:
                if (statetureta.milliseconds() > TURRET_STATE_DEBOUNCE_MS) {
                    LLResult result = limelight.limelight.getLatestResult();
                    if ((result == null || !result.isValid()) && !turetaToZero) {
                        turret.setPower(0);
                    } else {
                        List<LLResultTypes.FiducialResult> tags = result != null ? result.getFiducialResults() : null;
                        if ((tags == null || tags.isEmpty()) && !turetaToZero) {
                            turret.setPower(0);
                        } else {
                            LLResultTypes.FiducialResult targetTag = null;
                            if (tags != null) {
                                for (LLResultTypes.FiducialResult t : tags) {
                                    if (t.getFiducialId() == getAprilTagIdForTurret()) {
                                        targetTag = t;
                                        break;
                                    }
                                }
                            }
                            if (targetTag == null && !turetaToZero) {
                                turret.setPower(0);
                            } else {
                                double tx = targetTag != null ? targetTag.getTargetXDegrees() : 0;
                                if (Math.abs(tx) < TURRET_TX_DEADZONE_DEG && !turetaToZero) {
                                    turret.setPower(0);
                                } else {
                                    double power = tx * TURRET_TX_POWER_GAIN;
                                    int pos = turret.getCurrentPosition();
                                    if ((pos >= MAX_TICKS && power > 0) || (pos <= MIN_TICKS && power < 0)) {
                                        power = 0;
                                    }
                                    if (gamepad1.right_bumper) {
                                        targetPosition = wrapTicks(0);
                                        turetaToZero = true;
                                        power = pidfTurret(targetPosition, wrapTicks(turret.getCurrentPosition()));
                                        power = clamp(power, -1, 1);
                                    } else {
                                        power = Math.max(-TURRET_POWER_CLAMP, Math.min(TURRET_POWER_CLAMP, power));
                                        turetaToZero = false;
                                    }
                                    //TODO: revert
//                                    turret.setPower(power);
                                }
                            }
                        }
                    }
                    if (gamepad1.share) {
                        turretHoldTicks = turret.getCurrentPosition();
                        resumeFromLast = true;
                        statetureta.reset();
                        return 0;
                    }
                }
                break;
        }
        return stateTureta;
    }

    protected int runThrowStateMachine(int stateThrow, ElapsedTime throwTimer) {
        switch (stateThrow) {
            case 0:
                if (gamepad1.triangle) {
                    outtake.Angle.setPosition(s1);
                    timerToSee.reset();
                    indexer.PullUp();
                    stopIntake = false;
                    indexer.KeepInside();
                    if (FAR && (launcher.getVelocity() > 1500 && launcher.getVelocity() < 1600 || timerToSee.milliseconds() > 400)) {
                        indexer.OuttakePose1();
                        throwTimer.reset();
                        return 1;
                    } else if (!CLOSE && !FAR && (launcher.getVelocity() < 1550 && launcher.getVelocity() > 1450 || timerToSee.milliseconds() > 400)) {
                        indexer.OuttakePose1();
                        throwTimer.reset();
                        return 1;
                    } else if (CLOSE && (launcher.getVelocity() < 1200 && launcher.getVelocity() > 1100 || timerToSee.milliseconds() > 400)) {
                        indexer.OuttakePose1();
                        throwTimer.reset();
                        return 1;
                    }
                }
                break;
            case 1:
                if (FAR && throwTimer.milliseconds() > timerFar1) {
                    outtake.Angle.setPosition(s2);
                    indexer.OuttakePose2();
                    throwTimer.reset();
                    return 2;
                } else if (!CLOSE && !FAR && throwTimer.milliseconds() > timerClose) {
                    outtake.Angle.setPosition(s2);
                    indexer.OuttakePose2();
                    throwTimer.reset();
                    return 2;
                } else if (CLOSE && throwTimer.milliseconds() > timerClose) {
                    indexer.OuttakePose1();
                    throwTimer.reset();
                    return 2;
                }
                break;
            case 2:
                if (FAR && throwTimer.milliseconds() > timerFar2) {
                    outtake.Angle.setPosition(s3);
                    indexer.OuttakePose3();
                    throwTimer.reset();
                    return 3;
                } else if (!CLOSE && !FAR && throwTimer.milliseconds() > timerClose) {
                    indexer.OuttakePose3();
                    outtake.Angle.setPosition(s3);
                    throwTimer.reset();
                    return 3;
                } else if (CLOSE && throwTimer.milliseconds() > timerClose) {
                    indexer.OuttakePose1();
                    throwTimer.reset();
                    return 3;
                }
                break;
            case 3:
                if (throwTimer.milliseconds() > 400) {
                    indexer.StopPullUp();
                    stopIntake = false;
                    allBallsIn = false;
                    ballsRemoved = true;
                    timeThrow = timerToSee.seconds();
                    return 0;
                }
                break;
        }
        return stateThrow;
    }

    /**
     * Called before runCollectStateMachine each cycle. Override to update preset/detected case (e.g. TeleOpSortare).
     */
    protected void beforeCollectCycle() {}

    protected int runCollectStateMachine(int stateCollect, ElapsedTime collecttimer) {
        switch (stateCollect) {
            case 0:
                indexer.PickPose3();
                if (distance <= 68) {
                    indexer.PickPose2();
                    collecttimer.reset();
                    ballsRemoved = false;
                    return 1;
                }
                break;
            case 1:
                if (distance <= 69 && collecttimer.milliseconds() > COLLECT_DELAY_POSE1_TO_2_MS) {
                    indexer.PickPose1();
                    collecttimer.reset();
                    return 2;
                }
                break;
            case 2:
                if (distance <= 69 && collecttimer.milliseconds() > COLLECT_DELAY_POSE2_TO_3_MS) {
                    collecttimer.reset();
                    allBallsIn = true;
                    stopIntake = true;
                    gamepad1.rumble(RUMBLE_COLLECT_COMPLETE_MS);
                    return 3;
                }
                break;
            case 3:
                if (collecttimer.milliseconds() > COLLECT_DELAY_POSE2_TO_3_MS) {
                    collecttimer.reset();
                    return 4;
                }
                break;
            case 4:
                if (ballsRemoved) {
                    collecttimer.reset();
                    stopIntake = false;
                    allBallsIn = false;
                    return 0;
                }
                break;
        }
        return stateCollect;
    }

    protected void handleIntake(ElapsedTime collecttimer) {
        if (gamepad1.left_trigger > 0 && !ridic) {
            indexer.KeepInside();
        } else if (gamepad1.right_trigger > 0 && !ridic) {
            indexer.TakeOut();
        } else if (takeGreenBallOut.milliseconds() < COLLECT_EJECT_DURATION_MS && !ridic) {
            indexer.TakeOut();
            greencount = 1;
        } else if (takePurpleBallOut.milliseconds() < COLLECT_EJECT_DURATION_MS && !ridic) {
            indexer.TakeOut();
            purplecount = 2;
        } else if ((stopIntake && collecttimer.milliseconds() > COLLECT_STOP_INTAKE_DELAY_MS) || ridic) {
            indexer.Stop();
        } else if (!ridic) {
            indexer.Colect();
        }
    }

    protected void addPositionTelemetry() {
        telemetry.addData("X (inch)", "%.1f", follower.getPose().getX());
        telemetry.addData("Y (inch)", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.getHeading()));
    }

    protected double pidfTurret(int target, int current) {
        int error = wrapTicks(target - current);
        integralTurret += error;
        double derivative = error - lastErrorTurret;
        lastErrorTurret = error;
        return (TURRET_POS_P * error) + (TURRET_POS_I * integralTurret) + (TURRET_POS_D * derivative) + (TURRET_POS_F * Math.signum(error));
    }

    protected int wrapTicks(int ticks) {
        int range = MAX_TICKS * 2;
        while (ticks > MAX_TICKS) ticks -= range;
        while (ticks < -MAX_TICKS) ticks += range;
        return ticks;
    }

    protected double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
