package org.firstinspires.ftc.teamcode.OpModes.Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.OpmodesConstants.*;

@Configurable
@TeleOp
public class TeleOpRed extends BaseTeleOp {

    @Override
    protected Pose getStartPose() { return RED_START_POSE; }

    @Override
    protected Pose getParkPose() { return RED_PARK_POSE; }

    @Override
    protected int getParkHeadingDeg() { return RED_PARK_HEADING_DEG; }

    @Override
    protected int getTowerX() { return RED_TOWER_X; }

    @Override
    protected int getTowerY() { return RED_TOWER_Y; }

    @Override
    protected Pose getTurretOverridePose() { return RED_TURRET_OVERRIDE_POSE; }

    @Override
    protected int getAprilTagIdForTurret() { return ID_RT; }
}
