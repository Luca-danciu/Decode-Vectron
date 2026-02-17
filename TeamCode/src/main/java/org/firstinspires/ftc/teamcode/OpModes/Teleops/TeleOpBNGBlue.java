package org.firstinspires.ftc.teamcode.OpModes.Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.*;

@Configurable
@TeleOp
public class TeleOpBNGBlue extends BaseTeleOp {

    @Override
    protected Pose getStartPose() { return BNG_BLUE_START_POSE; }

    @Override
    protected Pose getParkPose() { return BLUE_PARK_POSE; }

    @Override
    protected int getParkHeadingDeg() { return BLUE_PARK_HEADING_DEG; }

    @Override
    protected int getTowerX() { return BNG_BLUE_TOWER_X; }

    @Override
    protected int getTowerY() { return BNG_BLUE_TOWER_Y; }

    @Override
    protected Pose getTurretOverridePose() { return BLUE_TURRET_OVERRIDE_POSE; }

    @Override
    protected int getAprilTagIdForTurret() { return ID_BT; }

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
}
