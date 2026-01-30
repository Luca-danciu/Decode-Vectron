package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class TurretSubsystem {
    private DcMotorEx turretMotor;
    private Follower follower; // Referință la Follower-ul Pedro Pathing

    private final double TICKS_PER_MOTOR_REV = 384.0; // GoBilda 435 RPM standard
    private final double GEAR_RATIO = 6.533;
    private final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;
    private final double TICKS_PER_DEGREE = TICKS_PER_TURRET_REV / 360.0;

    private final double MAX_ANGLE_DEGREES = 180.0; // Limita software ±180°
    private double currentTurretAngleDegrees = 0.0; // Track unghi relativ actual (software)

    private double targetX, targetY; // Punct fix în coordonate teren (inches)

    // PID basics (tunează!)
    private final double MAX_POWER = 1;

    public TurretSubsystem(HardwareMap hardwareMap, Follower follower,
                           double targetX, double targetY,
                           Pose startingRobotPose) {
        this.follower = follower;
        this.targetX = targetX;
        this.targetY = targetY;

        // Inițializează motorul
        turretMotor = hardwareMap.get(DcMotorEx.class, "Tureta");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD); // Ajustează direcția
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Important: setează starting pose-ul în Follower imediat!
        // (de obicei faci asta în OpMode init, dar aici ca exemplu)
        if (startingRobotPose != null) {
            follower.setStartingPose(startingRobotPose);
        }

        // Opțional: resetează turela la 0 la start (facing forward)
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(1);
    }

    // Normalizare unghi la -180..+180 grade
    private double normalizeAngle(double angleDegrees) {
        angleDegrees = angleDegrees % 360;
        if (angleDegrees > 180) angleDegrees -= 360;
        if (angleDegrees < -180) angleDegrees += 360;
        return angleDegrees;
    }

    // Calculează cea mai scurtă diferență de unghi (cu wrap-around)
    private double getShortestDelta(double current, double target) {
        double delta = normalizeAngle(target - current);
        if (Math.abs(delta) > 180) {
            delta -= Math.signum(delta) * 360;
        }
        return delta;
    }

    // Update principal – apelează-l în fiecare loop!
    public void update() {
        Pose currentPose = follower.getPose(); // x, y în inches, heading în radiani

        double robotHeadingDegrees = Math.toDegrees(currentPose.getHeading());

        // Unghi absolut spre target (din robot spre punct fix)
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();
        double absoluteTargetAngleDegrees = Math.toDegrees(Math.atan2(dy, dx));

        // Unghi relativ dorit pentru turelă
        double desiredTurretAngleDegrees = normalizeAngle(absoluteTargetAngleDegrees - robotHeadingDegrees);

        // Calculează delta scurt (cu logica de wrap la 180°)
        double deltaAngle = getShortestDelta(currentTurretAngleDegrees, desiredTurretAngleDegrees);

        // Update track software
        currentTurretAngleDegrees = normalizeAngle(currentTurretAngleDegrees + deltaAngle);

        // Calculează ticks țintă relativ la poziția curentă
        int currentTicks = turretMotor.getCurrentPosition();
        int deltaTicks = (int) (deltaAngle * TICKS_PER_DEGREE);
        int targetTicks = currentTicks + deltaTicks;

        // Limitează la ±180° (opțional, dar bun de avut)
        int maxTicks = (int) (MAX_ANGLE_DEGREES * TICKS_PER_DEGREE);
        if (Math.abs(targetTicks) > maxTicks * 1.1) { // mic buffer
            // Dacă e prea departe, forțează wrap (rar ar trebui să ajungă aici)
            targetTicks = (int) (Math.signum(targetTicks) * (360 * TICKS_PER_DEGREE - Math.abs(targetTicks)));
        }

        // Aplică
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(MAX_POWER);
    }

    public void stop() {
        turretMotor.setPower(0);
    }

    // Dacă vrei să schimbi target-ul dinamic
    public void setTarget(double x, double y) {
        targetX = x;
        targetY = y;
    }

    // Pentru debug
    public double getCurrentTurretAngle() {
        return currentTurretAngleDegrees;
    }
}

