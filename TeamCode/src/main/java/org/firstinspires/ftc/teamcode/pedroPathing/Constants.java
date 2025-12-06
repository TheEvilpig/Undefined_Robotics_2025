package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.HConst;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.253284)

            .forwardZeroPowerAcceleration(-35)
            .lateralZeroPowerAcceleration(-67)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.001, 0, 0))
            .centripetalScaling(0.00075);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(85.86)
            .yVelocity(68.8)

            .rightFrontMotorName(HConst.RIGHT_FRONT)
            .rightRearMotorName(HConst.RIGHT_BACK)
            .leftRearMotorName(HConst.LEFT_BACK)
            .leftFrontMotorName(HConst.LEFT_FRONT)

            .leftFrontMotorDirection(HConst.LF_DIR)
            .leftRearMotorDirection(HConst.LB_DIR)
            .rightFrontMotorDirection(HConst.RF_DIR)
            .rightRearMotorDirection(HConst.RB_DIR);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.1)
            .strafePodX(-2.35)

            .distanceUnit(HConst.PINPOINT_DISTANCE_UNIT)

            .hardwareMapName(HConst.PINPOINT_NAME)

            .encoderResolution(HConst.PINPOINT_RESOLUTION)

            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.7, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
