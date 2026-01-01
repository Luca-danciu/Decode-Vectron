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
            .xVelocity(73.908)  //74.917
            .yVelocity(58.436) //; 58.362 58.523
    ;

    //8.5
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.3)
            //16 cm
            .strafePodX(-3.15)
            //-8 cm
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    ;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.52)
            //-32.185  -42.123 -42.7202 -35.288
            .forwardZeroPowerAcceleration(-35.288)
            //-67.822  65.637 -64.604 -66.111
            .lateralZeroPowerAcceleration(-66.111)

            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2 , 0 , 0.02 , 0.027))
            .headingPIDFCoefficients(new PIDFCoefficients(1 , 0 , 0 , 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025 , 0 , 0.00001 , 0.6 , 0.01))
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.4 , 0 , 0.05 , 0.03))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4 , 0 , 0.05 , 0.03))
//            .headingPIDFCoefficients(new PIDFCoefficients(1.5 , 0 , 0.07, 0.05))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1 , 0 , 0, 0.035))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04 , 0 , 0.00001 , 0.6 , 0.01))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.04 , 0 , 0.00001 , 0.6 , 0.01))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04 , 0 , 0.00001 , 0.6 , 0.01))
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
