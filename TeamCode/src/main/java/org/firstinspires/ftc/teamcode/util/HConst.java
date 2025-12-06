package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class HConst {

    /*  CONFIG STRUCTURE
    EHUB:
        MOTORS:
            0: rf
            1: rb
            2: Transfer
            3: Intake

        SERVOS:
            0: rs



     */



    // ----------------------
    // MOTOR CONSTANTS
    // ----------------------

    // drivetrain motor names
    public static final String LEFT_FRONT = "lf";
    public static final String LEFT_BACK  = "lb";
    public static final String RIGHT_FRONT = "rf";
    public static final String RIGHT_BACK  = "rb";

    // drivetrain directions
    public static final DcMotor.Direction LF_DIR = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction LB_DIR = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction RF_DIR = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RB_DIR = DcMotor.Direction.FORWARD;

    // aux motors
    public static final String INTAKE = "Intake";
    public static final String OUTTAKE = "Outtake";
    public static final String TRANSFER = "Transfer";

    // aux directions
    public static final DcMotorSimple.Direction INTAKE_DIR = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction OUTTAKE_DIR = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction TRANSFER_DIR = DcMotorSimple.Direction.REVERSE;

    // ----------------------
    // PINPOINT ODOMETRY CONSTANTS
    // ----------------------

    // hardware name in config
    public static final String PINPOINT_NAME = "pinpoint";

    // pod offsets (in INCHES)
    public static final double PINPOINT_FORWARD_POD_Y = 6.1;
    public static final double PINPOINT_STRAFE_POD_X = -2.35;

    // unit
    public static final DistanceUnit PINPOINT_DISTANCE_UNIT = DistanceUnit.INCH;

    // resolution / driver settings
    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_RESOLUTION =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;


    // ----------------------
    // LIMELIGHT / CAMERA CONSTANTS
    // ----------------------

    // limelight
    public static final String LIMELIGHT = "Limelight";

    // ----------------------
    // SERVO CONSTANTS
    // ----------------------

    // servos
    public static final String R_BRAKE = "rs";
    public static final String L_BRAKE = "ls";

    private HConst() {} // prevent instantiation
}
