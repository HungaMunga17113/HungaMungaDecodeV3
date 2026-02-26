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

//Must connect once to robot wifi using usb c to a cable
public class Constants {
    //Panels Dashboard - http://192.168.43.1:8001/
    public static FollowerConstants followerConstants = new FollowerConstants()
            /*
            .forwardZeroPowerAcceleration(-30.327367609188478)
            .lateralZeroPowerAcceleration(-55.0414278486583)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0.01, 0.08))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05,0,0.001,0.008))
            .headingPIDFCoefficients(new PIDFCoefficients(2.24, 0, 0.08, 0.05)) //1.3
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.14, 0, 0.06, 0)) //1.1
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04,0.0,0.0001,0.6,0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.035,0,0.000067,0.6,0.02))
            .centripetalScaling(0.000575)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)

             */
            .mass(8.35);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3)
            .strafePodX(-3.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)

            //.xVelocity(90)
            //79.7965
            //.yVelocity(80)
            .useBrakeModeInTeleOp(true)
            //62.5067153089624
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}