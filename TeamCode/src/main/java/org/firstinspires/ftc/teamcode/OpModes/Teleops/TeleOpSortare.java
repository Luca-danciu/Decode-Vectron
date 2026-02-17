package org.firstinspires.ftc.teamcode.OpModes.Teleops;

import static org.firstinspires.ftc.teamcode.OpmodesConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp
public class TeleOpSortare extends BaseTeleOp {

    protected String presetCase = "";
    protected String detectedCase = "NONE";
    protected String previousPresetCase = null;

    @Override
    protected Pose getStartPose() {
        return BNG_BLUE_START_POSE;
    }

    @Override
    protected Pose getParkPose() {
        return BLUE_PARK_POSE;
    }

    @Override
    protected int getParkHeadingDeg() {
        return BLUE_PARK_HEADING_DEG;
    }

    @Override
    protected int getTowerX() {
        return BNG_BLUE_TOWER_X;
    }

    @Override
    protected int getTowerY() {
        return BNG_BLUE_TOWER_Y;
    }

    @Override
    protected Pose getTurretOverridePose() {
        return BLUE_TURRET_OVERRIDE_POSE;
    }

    @Override
    protected int getAprilTagIdForTurret() {
        return ID_BT;
    }

    @Override
    protected Pose getCircleOverridePose() {
        return new Pose(18, 119, Math.toRadians(145));
    }

    @Override
    protected void updateTowerFromDistance(double distInches) {
        if (distInches > 105) {
            towerX = 10;
        } else {
            towerX = BNG_BLUE_TOWER_X;
        }
    }

    @Override
    protected void beforeCollectCycle() {
        if (gamepad2.square) presetCase = "PPG";
        if (gamepad2.triangle) presetCase = "PGP";
        if (gamepad2.circle) presetCase = "GPP";
        if (gamepad2.dpad_up) detectedCase = "PPG";
        if (gamepad2.dpad_right) detectedCase = "PGP";
        if (gamepad2.dpad_left) detectedCase = "GPP";
        String tag = limelight.getAprilTag();
        if (tag != null && (tag.equals("GPP") || tag.equals("PGP") || tag.equals("PPG"))) {
            detectedCase = tag;
        }
    }

    @Override
    protected int runCollectStateMachine(int stateCollect, ElapsedTime collecttimer) {
        if (previousPresetCase != null && !previousPresetCase.equals(presetCase)) {
            previousPresetCase = presetCase;
            return 0;
        }
        previousPresetCase = presetCase;
        int stateCollectSelectiv = stateCollect;
        switch (stateCollectSelectiv) {
            case 0:
                ballsRemoved = false;
                if (detectedCase.equals("PPG")) {
                    switch (presetCase) {
                        case "PPG": indexer.PickPose1(); break;
                        case "PGP": indexer.PickPose3(); break;
                        case "GPP": indexer.PickPose2(); break;
                        default: indexer.PickPose3(); break;
                    }
                } else if (detectedCase.equals("PGP")) {
                    switch (presetCase) {
                        case "PPG": indexer.PickPose3(); break;
                        case "PGP": indexer.PickPose3(); break;
                        case "GPP": indexer.PickPose1(); break;
                        default: indexer.PickPose3(); break;
                    }
                } else if (detectedCase.equals("GPP")) {
                    switch (presetCase) {
                        case "PPG": indexer.PickPose1(); break;
                        case "PGP": indexer.PickPose2(); break;
                        case "GPP": indexer.PickPose3(); break;
                        default: indexer.PickPose3(); break;
                    }
                }
                if (distance < 64) {
                    stateCollectSelectiv = 1;
                    collecttimer.reset();
                }
                break;
            case 1:
                if (detectedCase.equals("PPG")) {
                    switch (presetCase) {
                        case "PPG": indexer.PickPose3(); break;
                        case "PGP": indexer.PickPose2(); break;
                        case "GPP": indexer.PickPose3(); break;
                        default: indexer.PickPose2(); break;
                    }
                } else if (detectedCase.equals("PGP")) {
                    switch (presetCase) {
                        case "PPG": indexer.PickPose2(); break;
                        case "PGP": indexer.PickPose1(); break;
                        case "GPP": indexer.PickPose2(); break;
                        default: indexer.PickPose2(); break;
                    }
                } else if (detectedCase.equals("GPP")) {
                    switch (presetCase) {
                        case "PPG": indexer.PickPose2(); break;
                        case "PGP": indexer.PickPose3(); break;
                        case "GPP": indexer.PickPose2(); break;
                        default: indexer.PickPose2(); break;
                    }
                }
                if (distance < 64 && collecttimer.milliseconds() > 300) {
                    stateCollectSelectiv = 2;
                    collecttimer.reset();
                }
                break;
            case 2:
                if (detectedCase.equals("PPG")) {
                    switch (presetCase) {
                        case "PPG": indexer.PickPose2(); break;
                        case "PGP": indexer.PickPose1(); break;
                        case "GPP": indexer.PickPose1(); break;
                        default: indexer.PickPose1(); break;
                    }
                } else if (detectedCase.equals("PGP")) {
                    switch (presetCase) {
                        case "PPG": indexer.PickPose1(); break;
                        case "PGP": indexer.PickPose2(); break;
                        case "GPP": indexer.PickPose3(); break;
                        default: indexer.PickPose1(); break;
                    }
                } else if (detectedCase.equals("GPP")) {
                    switch (presetCase) {
                        case "PPG": indexer.PickPose3(); break;
                        case "PGP": indexer.PickPose1(); break;
                        case "GPP": indexer.PickPose1(); break;
                        default: indexer.PickPose1(); break;
                    }
                }
                if (distance < 64 && collecttimer.milliseconds() > 300) {
                    stateCollectSelectiv = 3;
                    indexer.PickPose1();
                    collecttimer.reset();
                }
                break;
            case 3:
                if (ballsRemoved) {
                    stateCollectSelectiv = 0;
                }
                break;
        }
        return stateCollectSelectiv;
    }

    @Override
    protected void addPositionTelemetry() {
        super.addPositionTelemetry();
        telemetry.addData("Detected", detectedCase);
        telemetry.addData("Given", presetCase);
        telemetry.addData("sattecollect", lastCollectState);
    }
}
