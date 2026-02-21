package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
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

    // servo names
    public static final String TURRET1 = "t1";
    public static final String TURRET2 = "t2";
    public static final String TURRET3 = "t3";
    public static final String HOOD = "h";
    public static final String INTAKE_LEFT_SERVO = "iLiftLeft";
    public static final String INTAKE_RIGHT_SERVO = "iLiftRight";

    // drivetrain directions
    public static final DcMotor.Direction LF_DIR = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction LB_DIR = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction RF_DIR = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RB_DIR = DcMotor.Direction.FORWARD;

    // aux motors
    public static final String OUTTAKE1 = "outtake1";
    public static final String OUTTAKE2 = "outtake2";
    public static final String TRANSFER1 = "transfer1";
    public static final String TRANSFER2 = "transfer2";
    public static final String COLOR = "Color";

    // aux directions
    public static final DcMotorSimple.Direction OUTTAKE1_DIR = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction OUTTAKE2_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction TRANSFER1_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction TRANSFER2_DIR = DcMotorSimple.Direction.REVERSE;


    //servo directions
    public static final Servo.Direction TURRET_DIR = Servo.Direction.FORWARD;

    public static final Servo.Direction INTAKE1_SERV_DIR = Servo.Direction.FORWARD;
    public static final Servo.Direction INTAKE2_SERV_DIR = Servo.Direction.REVERSE;
    public static final Servo.Direction HOOD_DIR = Servo.Direction.FORWARD;


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

    //height of target, 30 inches in meters
    public static final double LLH2 = 0.762;

    //height of limelight in meters, 8 inches
    public static final double LLH1 = 0.2032;

    //mounted angle of limelight in degrees
    public static final double LLANGL = 23.0;

    // ----------------------
    // SERVO CONSTANTS
    // ----------------------

    // servos
    public static final String R_BRAKE = "rs";
    public static final String L_BRAKE = "ls";

    public static final String HOLD = "Hold";

    // servo positions
    public static final double L_BRAKE_UP = 0.83;
    public static final double L_BRAKE_DOWN = 1;

    public static final double R_BRAKE_UP = 0.19;
    public static final double R_BRAKE_DOWN = 0;

    public static final double HOLD_ACTIVE = 0;
    public static final double HOLD_INACTIVE = 1;

    public static final double INTAKE1UP = 0;
    public static final double INTAKE1DOWN = 1;
    public static final double INTAKE2UP = 0;
    public static final double INTAKE2DOWN = 1;

    // ----------------------
    // TELEOP ENUMS
    // ----------------------

    public enum DriveTrainMode {
        GAMEPAD_ROBOT_CENTRIC,
        GAMEPAD_FIELD_CENTRIC,
        AUTO_TARGET_GOAL,
        STOP
    }

    private HConst() {} // prevent instantiation
}