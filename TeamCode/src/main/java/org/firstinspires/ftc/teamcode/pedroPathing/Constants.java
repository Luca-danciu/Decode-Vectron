package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("RR")
            .leftRearMotorName("RL")
            .leftFrontMotorName("FL")
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(86.32274)
            .yVelocity(64.1973);

    //8.5
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.1299213)
            .strafePodX(2.95276)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.8)
            .forwardZeroPowerAcceleration(-2671.650033)
            .lateralZeroPowerAcceleration(-54.2674)
//            .forwardZeroPowerAcceleration(-20508.245988)
//            .lateralZeroPowerAcceleration(-74.929417451)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12 , 0 , 0.001 , 0.005))
            .headingPIDFCoefficients(new PIDFCoefficients(1 , 0 , 0, 0.000005))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04 , 0 , 0.00001 , 0.6 , 0.01))
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.035 , 0 , 0.04 , 0.004))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients( 0.035 , 0 , 0.04 , 0.004 ))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.7 , 0 , 0.00001 , 0.005))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.7 , 0 , 0.00001 , 0.005))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.3, 0 , 0.00001 , 0.004 ))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.3, 0 , 0.00001 , 0.004 ))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 10, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
