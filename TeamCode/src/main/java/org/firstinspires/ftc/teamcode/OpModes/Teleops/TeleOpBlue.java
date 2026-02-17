package org.firstinspires.ftc.teamcode.OpModes.Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.*;

@Configurable
@TeleOp
public class TeleOpBlue extends BaseTeleOp {

    @Override
    protected Pose getStartPose() { return BLUE_START_POSE; }

    @Override
    protected Pose getParkPose() { return BLUE_PARK_POSE; }

    @Override
    protected int getParkHeadingDeg() { return BLUE_PARK_HEADING_DEG; }

    @Override
    protected int getTowerX() { return BLUE_TOWER_X; }

    @Override
    protected int getTowerY() { return BLUE_TOWER_Y; }

    @Override
    protected Pose getTurretOverridePose() { return BLUE_TURRET_OVERRIDE_POSE; }

    @Override
    protected int getAprilTagIdForTurret() { return ID_BT; }
}
