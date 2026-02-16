package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Central repository for all robot constants: hardware IDs, tuning values,
 * physical dimensions, timings, and thresholds.
 * <p>
 * Organized by subsystem for easy maintenance. All magic numbers should be
 * defined here rather than scattered throughout the codebase.
 * Hover over any constant to see its purpose and usage.
 */
@Configurable
public final class Constants {

    // =========================================================================
    // APRIL TAG / VISION
    // =========================================================================

    /** AprilTag ID for Purple-Green-Purple ball configuration on the field. */
    public static final int ID_PGP = 22;
    /** AprilTag ID for Purple-Purple-Green ball configuration on the field. */
    public static final int ID_PPG = 23;
    /** AprilTag ID for Green-Purple-Purple ball configuration on the field. */
    public static final int ID_GPP = 21;
    /** AprilTag ID for Blue Tower (BT) - turret targets this on blue alliance. */
    public static final int ID_BT = 20;
    /** AprilTag ID for Red Tower (RT) - turret targets this on red alliance. */
    public static final int ID_RT = 24;

    /**
     * Calibration factor for converting webcam range reading to meters.
     * Applied as: {@code distanceMeters = rawRange * WEBCAM_RANGE_MODIFIER}.
     */
    public static final double WEBCAM_RANGE_MODIFIER = 0.030099;

    /**
     * Webcam resolution in pixels (width x height).
     * Affects AprilTag detection quality and processing speed.
     */
    public static final Size WEBCAM_SIZE = new Size(640, 480);

    /**
     * Physical size of each AprilTag in meters.
     * Used by the vision pipeline for distance/pose estimation accuracy.
     */
    public static final double APRILTAG_PHYSICAL_SIZE_METERS = 0.16;

    /** Limelight pipeline index (0-9) used for AprilTag/fiducial detection. */
    public static final int LIMELIGHT_APRILTAG_PIPELINE = 8;

    /**
     * Limelight distance formula constant.
     * Distance is computed as: {@code LIMELIGHT_DISTANCE_SCALE / targetArea}.
     */
    public static final double LIMELIGHT_DISTANCE_SCALE = 28197.31;

    /**
     * Divisor to convert Limelight raw distance to meters.
     * Final distance = {@code rawDistance / LIMELIGHT_DISTANCE_DIVISOR}.
     */
    public static final int LIMELIGHT_DISTANCE_DIVISOR = 10000;

    /**
     * Color sensor amplification gain (0–1023).
     * Higher values brighten the reading; adjust if detection is unstable.
     */
    public static final float COLOR_SENSOR_GAIN = 100f;

    /** Minimum normalized red for purple ball detection. */
    public static final double PURPLE_RED_MIN = 1.75;
    /** Maximum normalized red for purple ball detection. */
    public static final double PURPLE_RED_MAX = 2.08;
    /** Minimum normalized green for purple ball detection. */
    public static final double PURPLE_GREEN_MIN = 2.65;
    /** Maximum normalized green for purple ball detection. */
    public static final double PURPLE_GREEN_MAX = 3.01;
    /** Minimum normalized blue for purple ball detection. */
    public static final double PURPLE_BLUE_MIN = 1.76;
    /** Maximum normalized blue for purple ball detection. */
    public static final double PURPLE_BLUE_MAX = 3.01;

    /** Minimum normalized red for green ball detection. */
    public static final double GREEN_RED_MIN = 1.0;
    /** Maximum normalized red for green ball detection. */
    public static final double GREEN_RED_MAX = 1.73;
    /** Minimum normalized green for green ball detection. */
    public static final double GREEN_GREEN_MIN = 3.01;
    /** Maximum normalized green for green ball detection. */
    public static final double GREEN_GREEN_MAX = 3.8;
    /** Minimum normalized blue for green ball detection. */
    public static final double GREEN_BLUE_MIN = 2.4;
    /** Maximum normalized blue for green ball detection. */
    public static final double GREEN_BLUE_MAX = 2.98;

    // =========================================================================
    // TOWER TARGET POSITIONS (field coordinates for turret aiming)
    // =========================================================================

    /** Red alliance tower X coordinate (field inches). Used for turret angle calculation. */
    public static final int RED_TOWER_X = 129;
    /** Red alliance tower Y coordinate (field inches). Used for turret angle calculation. */
    public static final int RED_TOWER_Y = 130;

    /** Blue alliance tower X coordinate (field inches). Used for turret angle calculation. */
    public static final int BLUE_TOWER_X = 14;
    /** Blue alliance tower Y coordinate (field inches). Used for turret angle calculation. */
    public static final int BLUE_TOWER_Y = 138;

    /** BNG Blue variant tower X coordinate (field inches). */
    public static final int BNG_BLUE_TOWER_X = 12;
    /** BNG Blue variant tower Y coordinate (field inches). */
    public static final int BNG_BLUE_TOWER_Y = 138;

    // =========================================================================
    // TURRET
    // =========================================================================

    /**
     * Encoder ticks per degree of turret rotation in manual (joystick) aiming mode.
     * Used to convert desired angle to motor target position.
     */
    public static final double TURRET_TICKS_PER_DEGREE = 6.071111;

    /**
     * Encoder ticks per degree when using Limelight tracking mode.
     * May differ from manual mode due to different gearing or calibration.
     */
    public static final double TURRET_TICKS_PER_DEGREE_LIMELIGHT = 6.533;

    /** Turret proportional gain (P) - response to angle error. Increase for faster correction. */
    public static double TURRET_KP = 0.005;
    /** Turret integral gain (I) - eliminates steady-state error. Usually 0 for turret. */
    public static double TURRET_KI = 0.0;
    /** Turret derivative gain (D) - reduces overshoot and oscillation. */
    public static double TURRET_KD = 0.0002;
    /** Turret feedforward gain (F) - base power when error exists. */
    public static double TURRET_KF = 0.05;

    /** Turret position PID proportional gain - used when holding zero/position. */
    public static double TURRET_POS_P = 0.006;
    /** Turret position PID integral gain. */
    public static double TURRET_POS_I = 0.0;
    /** Turret position PID derivative gain. */
    public static double TURRET_POS_D = 0;
    /** Turret position PID feedforward gain - provides holding torque. */
    public static double TURRET_POS_F = 0.1;
    /** Auto turret F when moving to score (higher for responsiveness). */
    public static final double AUTO_TURRET_POS_F_MOVING = 0.3;
    /** Auto turret F when holding position after score. */
    public static final double AUTO_TURRET_POS_F_HOLDING = 0.1;

    /**
     * Limelight tx deadzone in degrees.
     * If target is within ±this angle of center, turret stops moving to avoid jitter.
     */
    public static final double TURRET_TX_DEADZONE_DEG = 1.0;

    /**
     * Multiplier converting Limelight tx angle (degrees) to turret motor power.
     * power = tx * TURRET_TX_POWER_GAIN. Increase for faster tracking.
     */
    public static final double TURRET_TX_POWER_GAIN = 0.03;

    /**
     * Max turret power when manually tracking (not zeroing).
     * Clamps output to ±TURRET_POWER_CLAMP to prevent aggressive corrections.
     */
    public static final double TURRET_POWER_CLAMP = 0.6;

    /**
     * Debounce time in ms before allowing turret state switch (share button).
     * Prevents accidental mode changes from brief button taps.
     */
    public static final int TURRET_STATE_DEBOUNCE_MS = 400;

    /**
     * Step size in degrees when adjusting turret offset via d-pad.
     * Each press adds or subtracts this from the current offset.
     */
    public static final double TURRET_OFFSET_STEP = 0.25;

    // =========================================================================
    // LAUNCHER / FLYWHEEL
    // =========================================================================

    /**
     * Launcher motor encoder counts per revolution (CPR).
     * Used to convert encoder delta to RPM: {@code RPM = (delta * 60) / (CPR * time)}.
     */
    public static final double LAUNCHER_CPR = 18.666;

    /** Launcher PIDF proportional gain. Increase for faster response to RPM error. */
    public static double LAUNCHER_KP = 0.003;
    /** Launcher PIDF integral gain. Helps eliminate steady-state RPM error. */
    public static double LAUNCHER_KI = 0.000007;
    /** Launcher PIDF derivative gain. Reduces overshoot when approaching target RPM. */
    public static double LAUNCHER_KD = 0.00007;
    /** Launcher PIDF feedforward. Adds power proportional to target RPM for better tracking. */
    public static double LAUNCHER_KF = 0.0001468;

    /** Default launcher power when using manual (non-PID) control. */
    public static final double LAUNCHER_DEFAULT_POWER = 0.9;

    /** Maximum allowed launcher RPM. Requests above this are clamped for safety. */
    public static final int LAUNCHER_MAX_RPM = 12000;

    /**
     * Minimum RPM threshold. Below this, launcher motor is stopped entirely
     * (no PID output) to avoid unnecessary power draw and wear.
     */
    public static final int LAUNCHER_MIN_ACTIVE_RPM = 50;

    /**
     * RPM tolerance for "at speed" check.
     * Ball is pushed when: {@code |measuredRPM - targetRPM| <= LAUNCHER_RPM_TOLERANCE}.
     */
    public static final int LAUNCHER_RPM_TOLERANCE = 100;

    /**
     * PIDF deadzone - if RPM error is below this, error is treated as 0.
     * Prevents motor chatter when nearly at target RPM.
     */
    public static final int LAUNCHER_ERROR_DEADZONE = 40;

    /**
     * Integral anti-windup limit.
     * Integral term is clamped to ±this value to prevent windup during long errors.
     */
    public static final int LAUNCHER_INTEGRAL_LIMIT = 5000;

    /**
     * PIDF control loop period in seconds.
     * Loop runs every LAUNCHER_DT seconds to compute new motor power.
     */
    public static final double LAUNCHER_DT = 0.02;

    /** Target RPM when distance to goal is &lt; 1.2 m (near zone). */
    public static final int LAUNCHER_RPM_NEAR = 3200;
    /** Target RPM when distance is 1.2–1.9 m (close zone). */
    public static final int LAUNCHER_RPM_CLOSE = 3400;
    /** Target RPM when distance is 1.9–3 m (mid zone). */
    public static final int LAUNCHER_RPM_MID = 3500;
    /** Target RPM when distance is 3–4.5 m (mid-far zone). */
    public static final int LAUNCHER_RPM_MID_FAR = 3700;
    /** Target RPM when distance is 4.5–8 m (far zone). */
    public static final int LAUNCHER_RPM_FAR = 4000;
    /** Target RPM when distance is &gt; 8 m (max zone). */
    public static final int LAUNCHER_RPM_MAX = 4800;

    /** Default outtake servo position when not actively shooting. */
    public static final double LAUNCHER_SERVO_DEFAULT = 0.28;

    // =========================================================================
    // DISTANCE ZONES (meters) - Outtake angle & RPM selection
    // =========================================================================

    /** Distance threshold (m). Below this = near zone; uses LAUNCHER_RPM_NEAR. */
    public static final double DIST_ZONE_NEAR = 1.2;
    /** Distance threshold (m). Defines boundary between near and close zones. */
    public static final double DIST_ZONE_CLOSE = 1.9;
    /** Distance threshold (m). Defines boundary between close and mid zones. */
    public static final double DIST_ZONE_MID = 3.0;
    /** Distance threshold (m). Defines boundary between mid and mid-far zones. */
    public static final double DIST_ZONE_MID_FAR = 4.5;
    /** Distance threshold (m). Above this = far zone; uses LAUNCHER_RPM_FAR or MAX. */
    public static final double DIST_ZONE_FAR = 8.0;

    // =========================================================================
    // OUTTAKE SERVO ANGLES (position 0–1)
    // =========================================================================

    /** Outtake angle servo position for near-distance shots (&lt; 1.2 m). */
    public static final double OUTTAKE_ANGLE_NEAR = 0.5;
    /** Outtake angle servo position for close-distance shots (1.2–1.9 m). */
    public static final double OUTTAKE_ANGLE_CLOSE = 0.67;
    /** Outtake angle servo position for mid-distance shots (1.9–4.5 m). */
    public static final double OUTTAKE_ANGLE_MID = 0.7;
    /** Outtake angle servo position for far-distance shots (&gt; 4.5 m). */
    public static final double OUTTAKE_ANGLE_FAR = 0.84;

    /** Minimum allowed outtake angle (servo position). Physical lower bound. */
    public static final double OUTTAKE_ANGLE_MIN = 0.46;
    /** Maximum allowed outtake angle (servo position). Physical upper bound. */
    public static final double OUTTAKE_ANGLE_MAX = 0.83;

    // =========================================================================
    // INDEXER
    // =========================================================================

    /** Intake motor power when keeping balls inside (light reverse). */
    public static final double INTAKE_POWER_KEEP_INSIDE = -0.7;
    /** Intake motor power for full collection (max reverse). */
    public static final double INTAKE_POWER_COLLECT = -1.0;
    /** Intake motor power for ejecting balls (full forward). */
    public static final double INTAKE_POWER_TAKE_OUT = 1.0;
    /** Intake motor power for partial eject (e.g. one ball at a time). */
    public static final double INTAKE_POWER_TAKE_OUT_BIT = 0.5;

    /** Indexer servo position for first pick slot (lowest position). */
    public static final double INDEXER_PICK_POSE_1 = 0.025;
    /** Indexer servo position for second pick slot (middle). */
    public static final double INDEXER_PICK_POSE_2 = 0.395;
    /** Indexer servo position for third pick slot (highest position). */
    public static final double INDEXER_PICK_POSE_3 = 0.785;

    /** Indexer servo position for first ball to fire (closest to launcher). */
    public static final double INDEXER_OUTTAKE_POSE_1 = 0.21;
    /** Indexer servo position for second ball to fire. */
    public static final double INDEXER_OUTTAKE_POSE_2 = 0.58;
    /** Indexer servo position for third ball to fire (farthest from launcher). */
    public static final double INDEXER_OUTTAKE_POSE_3 = 0.97;

    /** Banana/pusher servo position when pushing ball (red alliance). */
    public static final double INDEXER_PUSH_POSITION = 0.3;
    /** Banana/pusher servo position when pushing ball (blue alliance). */
    public static final double INDEXER_PUSH_BLUE_POSITION = 0.36;
    /** Banana/pusher servo position when retracted (rest/ready for next push). */
    public static final double INDEXER_DOWN_POSITION = 0.7;

    // =========================================================================
    // ASCENT (Climber)
    // =========================================================================

    /** Climber grip servo position when bar is gripped (closed). */
    public static final double ASCENT_GRIP_POSITION = 0;
    /** Climber grip servo position when bar is released (open). */
    public static final double ASCENT_RELEASE_POSITION = 0.4;

    /** Ascent lift power multiplier for descending (D) motors. */
    public static final double ASCENT_POWER_D = 1.0;
    /** Ascent lift power multiplier for ascending (S) motors. */
    public static final double ASCENT_POWER_S = 1.0;

    // =========================================================================
    // COLLECTION STATE MACHINE
    // =========================================================================

    /**
     * Distance threshold in mm. When distance sensor reads ≤ this, a ball is considered present.
     * Used to advance through PickPose1 → 2 → 3.
     */
    public static final int COLLECT_DISTANCE_THRESHOLD_MM = 70;

    /** Delay in ms to wait before moving from PickPose1 to PickPose2. Allows ball to settle. */
    public static final int COLLECT_DELAY_POSE1_TO_2_MS = 600;
    /** Delay in ms to wait before moving from PickPose2 to PickPose3. */
    public static final int COLLECT_DELAY_POSE2_TO_3_MS = 300;
    /** Delay in ms after all balls in before TakeOUT is allowed. Prevents premature ejection. */
    public static final int COLLECT_DELAY_BEFORE_TAKEOUT_MS = 300;
    /** Delay in ms after collect complete before intake motor stops. Clears any residual ball. */
    public static final int COLLECT_STOP_INTAKE_DELAY_MS = 700;
    /** Duration in ms for TakeGreenBallOut / TakePurpleBallOut intake ejection pulses. */
    public static final int COLLECT_EJECT_DURATION_MS = 400;

    /** Gamepad rumble duration (ms) when 3 balls successfully collected. */
    public static final int RUMBLE_COLLECT_COMPLETE_MS = 300;
    /** Gamepad rumble duration (ms) when selective collect (GPP/PGP/PPG) is complete. */
    public static final int RUMBLE_SELECTIVE_COMPLETE_MS = 200;

    // =========================================================================
    // THROW STATE MACHINE
    // =========================================================================

    /** Delay in ms to wait for launcher RPM before pushing first ball. Ensures flywheel is at speed. */
    public static final int THROW_RPM_WAIT_MS = 400;
    /** Delay in ms between push command and indexer returning to down position. */
    public static final int THROW_PUSH_TO_DOWN_MS = 200;
    /** Delay in ms between changing indexer pose when firing multiple balls. */
    public static final int THROW_POSE_CHANGE_MS = 200;
    /** Delay in ms after last ball thrown before resetting throw state machine. */
    public static final int THROW_RESET_DELAY_MS = 400;

    // =========================================================================
    // DRIVETRAIN
    // =========================================================================

    /**
     * Gamepad stick deadzone. Joystick values with magnitude &lt; this are treated as 0.
     * Reduces drift from analog stick noise.
     */
    public static final double DRIVE_STICK_DEADZONE = 0.05;

    /**
     * Power scaling factor for mecanum drive.
     * Output power = (computed power) / denominator * DRIVE_POWER_SCALE.
     */
    public static final double DRIVE_POWER_SCALE = 2.0;

    /** Step size when adjusting launcher power via d-pad (TeleOp manual mode). */
    public static final double LAUNCHER_POWER_ADJUST_STEP = 0.1;

    /** Min time (s) used in derivative term to avoid division by zero. */
    public static final double LAUNCHER_PID_DERIVATIVE_TIME_MIN = 0.01;

    /** Default turret PID power during auto tower paths (0–1). */
    public static final double AUTO_TURRET_POWER = 0.8;

    /** Default turret tx offset in degrees. */
    public static final double TURRET_TX_OFFSET_DEFAULT = 0.0;

    /** Number of consecutive stable color reads required before confirming ball color. */
    public static final int COLOR_SENSOR_REQUIRED_STABLE_READS = 3;

    // =========================================================================
    // PATH FOLLOWER (pedroPathing) - kP, kV, kA, velocities, etc.
    // =========================================================================

    /** Mecanum max power (0–1). */
    public static final double PATH_MAX_POWER = 1.0;
    /** Mecanum X velocity (inches/s) - forward/back. */
    public static final double PATH_X_VELOCITY = 73.908;
    /** Mecanum Y velocity (inches/s) - strafe. */
    public static final double PATH_Y_VELOCITY = 58.436;

    /** Pinpoint forward pod Y offset (inches). */
    public static final double PATH_FORWARD_POD_Y = 6.3;
    /** Pinpoint strafe pod X offset (inches). */
    public static final double PATH_STRAFE_POD_X = -3.15;

    /** Robot mass (kg) for path follower feedforward. */
    public static final double PATH_MASS = 9.52;
    /** Forward zero-power acceleration (in/s²). */
    public static final double PATH_FORWARD_ZERO_POWER_ACCEL = -35.288;
    /** Lateral zero-power acceleration (in/s²). */
    public static final double PATH_LATERAL_ZERO_POWER_ACCEL = -66.111;

    /** Translational PIDF kP – position error gain. */
    public static final double PATH_TRANSLATIONAL_KP = 0.2;
    /** Translational PIDF kI. */
    public static final double PATH_TRANSLATIONAL_KI = 0;
    /** Translational PIDF kD. */
    public static final double PATH_TRANSLATIONAL_KD = 0.02;
    /** Translational PIDF kF – feedforward. */
    public static final double PATH_TRANSLATIONAL_KF = 0.027;

    /** Heading PIDF kP. */
    public static final double PATH_HEADING_KP = 1;
    /** Heading PIDF kI. */
    public static final double PATH_HEADING_KI = 0;
    /** Heading PIDF kD. */
    public static final double PATH_HEADING_KD = 0;
    /** Heading PIDF kF. */
    public static final double PATH_HEADING_KF = 0.01;

    /** Drive (Filtered) PIDF kP. */
    public static final double PATH_DRIVE_KP = 0.025;
    /** Drive PIDF kI. */
    public static final double PATH_DRIVE_KI = 0;
    /** Drive PIDF kD. */
    public static final double PATH_DRIVE_KD = 0.00001;
    /** Drive PIDF kV – velocity feedforward. */
    public static final double PATH_DRIVE_KV = 0.6;
    /** Drive PIDF kA – acceleration feedforward. */
    public static final double PATH_DRIVE_KA = 0.01;

    /** Path constraints: max velocity. */
    public static final double PATH_CONSTRAINT_MAX_VELOCITY = 0.99;
    /** Path constraints: max acceleration. */
    public static final double PATH_CONSTRAINT_MAX_ACCELERATION = 100;
    /** Path constraints: max angular velocity. */
    public static final double PATH_CONSTRAINT_MAX_ANGULAR_VELOCITY = 10;
    /** Path constraints: max angular acceleration. */
    public static final double PATH_CONSTRAINT_MAX_ANGULAR_ACCELERATION = 1;

    /** Launcher motor power when "charging" (reverse spin-up before shot). */
    public static final double LAUNCHER_CHARGE_POWER = -0.8;

    // =========================================================================
    // AUTO / PARCARE (Parking) POSES
    // =========================================================================

    /** Red alliance starting pose: x, y (inches), heading (radians). Used at TeleOp init. */
    public static final Pose RED_START_POSE = new Pose(86, 117, Math.toRadians(0));
    /** Red alliance pose override when left bumper held. Positions robot for turret aim. */
    public static final Pose RED_TURRET_OVERRIDE_POSE = new Pose(105, 140, Math.toRadians(0));
    /** Red alliance park destination (x, y) when cross button pressed. */
    public static final Pose RED_PARK_POSE = new Pose(33, 33.4);
    /** Red alliance park heading in degrees. Robot faces this angle when parked. */
    public static final int RED_PARK_HEADING_DEG = 45;

    /** Blue alliance starting pose: x, y (inches), heading (radians). */
    public static final Pose BLUE_START_POSE = new Pose(57, 113, Math.toRadians(180));
    /** Blue alliance pose override when left bumper held. */
    public static final Pose BLUE_TURRET_OVERRIDE_POSE = new Pose(39, 140, Math.toRadians(180));
    /** Blue alliance park destination (x, y). */
    public static final Pose BLUE_PARK_POSE = new Pose(111, 29.5);
    /** Blue alliance park heading in degrees. */
    public static final int BLUE_PARK_HEADING_DEG = 135;

    /** BNG Blue variant starting pose. */
    public static final Pose BNG_BLUE_START_POSE = new Pose(40, 12, Math.toRadians(180));
    /**
     * Park heading interpolation blend factor (0–1).
     * Controls how quickly heading transitions toward park angle during path.
     */
    public static final double PARK_HEADING_BLEND = 0.8;

    // =========================================================================
    // AUTONOMOUS-SPECIFIC CONSTANTS
    // =========================================================================

    /** Tighter ball detection threshold (mm) used in some autos. */
    public static final int AUTO_COLLECT_DISTANCE_MM_TIGHT = 60;
    /** Faster collect delay (ms) from PickPose1 to PickPose2 in some autos. */
    public static final int AUTO_COLLECT_DELAY_POSE1_TO_2_MS_FAST = 500;
    /** Timeout (s) while driving to collect before giving up. */
    public static final double AUTO_COLLECT_TIMEOUT_DRIVING_SEC = 1.5;
    /** Delay (s) before reading AprilTag to allow camera to stabilize. */
    public static final double AUTO_TAG_DETECTION_DELAY_SEC = 0.05;
    /** Delay (s) before starting score path in BNG combo. */
    public static final double AUTO_BNG_SCORE_DELAY_SEC = 0.5;

    /** Auto throw: extended RPM wait (ms) before first push. Some autos use looser tolerance. */
    public static final int AUTO_THROW_RPM_WAIT_MS_EXTENDED = 700;
    /** Auto throw: mid RPM wait (ms) before push. */
    public static final int AUTO_THROW_RPM_WAIT_MS_MID = 600;
    /** Auto throw: alternate RPM wait (ms) for subsequent balls. */
    public static final int AUTO_THROW_RPM_WAIT_MS_ALT = 500;
    /** Auto throw: looser RPM tolerance (high) for "at speed" check. */
    public static final int AUTO_THROW_RPM_TOLERANCE_HIGH = 150;
    /** Auto throw: looser RPM tolerance (low) for "at speed" check. */
    public static final int AUTO_THROW_RPM_TOLERANCE_LOW = 50;

    /** Auto outtake angle for tower-style autos (close shot). */
    public static final double AUTO_OUTTAKE_ANGLE_TOWER = 0.53;
    /** Auto outtake angle for mid-range tower shots. */
    public static final double AUTO_OUTTAKE_ANGLE_MID = 0.72;
    /** Auto outtake angle default (used when no specific angle). */
    public static final double AUTO_OUTTAKE_ANGLE_DEFAULT = 0.76;
    /** Auto outtake angle for triangle/small triangle autos (far shot). */
    public static final double AUTO_OUTTAKE_ANGLE_TRIANGLE = 0.85;

    /** Default auto launcher RPM. */
    public static final int AUTO_LAUNCHER_RPM_DEFAULT = 3500;
    /** Auto launcher RPM for triangle/small triangle paths. */
    public static final int AUTO_LAUNCHER_RPM_TRIANGLE = 4900;
    /** Auto launcher RPM for BNG combo (first shot). */
    public static final int AUTO_LAUNCHER_RPM_BNG = 5300;
    /** Auto launcher RPM for BNG combo (subsequent shots). */
    public static final int AUTO_LAUNCHER_RPM_BNG_SECOND = 3600;
    /** Auto launcher RPM for far shot (e.g. last pickup). */
    public static final int AUTO_LAUNCHER_RPM_FAR = 2800;
    /** Auto launcher RPM for close shot (e.g. Blue tower last pickup). */
    public static final int AUTO_LAUNCHER_RPM_CLOSE_SHOT = 3000;

    /** Auto path follow speed (0–1) for slow segments. */
    public static final double AUTO_PATH_SPEED_SLOW = 0.6;
    /** Auto path follow speed (0–1) for mid segments. */
    public static final double AUTO_PATH_SPEED_MID = 0.7;
    /** Auto path follow speed (0–1) for fast segments. */
    public static final double AUTO_PATH_SPEED_FAST = 0.8;

    // =========================================================================
    // LEGACY / TUNING (kept for Configurables compatibility)
    // =========================================================================

    /** @deprecated Use {@link #LAUNCHER_RPM_NEAR} etc. instead. Legacy preset RPM tier 1. */
    public static double targetR1 = 3400;
    /** @deprecated Use {@link #LAUNCHER_RPM_CLOSE} etc. instead. Legacy preset RPM tier 2. */
    public static double targetR2 = 3600;
    /** @deprecated Use {@link #LAUNCHER_RPM_MID} etc. instead. Legacy preset RPM tier 3. */
    public static double targetR3 = 4200;
    /** @deprecated Use {@link #LAUNCHER_RPM_MAX} etc. instead. Legacy preset RPM tier 4. */
    public static double targetR4 = 4800;

    /** @deprecated Use {@link #COLLECT_DELAY_POSE1_TO_2_MS} etc. instead. Legacy timing interval 1. */
    public static double interval1 = 0.552;
    /** @deprecated Legacy timing interval 2. */
    public static double interval2 = 0.7;
    /** @deprecated Legacy timing interval 3. */
    public static double interval3 = 0.8;
}
