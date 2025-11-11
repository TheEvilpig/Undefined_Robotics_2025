package org.firstinspires.ftc.teamcode.roboCode.tele;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name="Main_TeleOp", group="Linear OpMode")
public class mainTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime;
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive,
        intake, outtake, transfer;

    private Servo b1, b2;
    private GoBildaPinpointDriver pinpoint;

    boolean aPressed = false;
    boolean runningDown = false;
    boolean runningUp = false;
    ElapsedTime sequenceTimer = new ElapsedTime();

    private Follower follower;

    private double rl = 0.5;

    double B2U = 0.19;
    double B2C = 0;

    double B1U = 0.83;
    double B1C = 1;


    private boolean reverse = false;
    private boolean bumper = false;

    private double adjust = 0.7;
    private double flip = 1;

    boolean dp = false;
    boolean du = false;

    boolean ga = false;

    boolean servopos = false;

    boolean downseq = false;

    boolean upseq = true;

    // Add this field at the top of your class (outside all methods)
    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean transferActive = false;
    private double transferDuration = 0.3; // seconds


    Orientation angles;
    @Override
    public void runOpMode() {

        //<editor-fold desc="INIT">
        follower = Constants.createFollower(hardwareMap);

        ElapsedTime runtime = new ElapsedTime();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        follower.setStartingPose(new Pose(72,72,0));

        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rb");

        intake = hardwareMap.get(DcMotor.class,"Intake");
        outtake = hardwareMap.get(DcMotor.class,"Outtake");
        transfer = hardwareMap.get(DcMotor.class,"Transfer");

        b2 = hardwareMap.get(Servo.class, "ls");
        b1 = hardwareMap.get(Servo.class, "rs");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");

        //</editor-fold>

        b1.setPosition(B1U);
        b2.setPosition(B2U);

        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updateDrivetrain();

            updateAuxiliaryMotors();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("===Testing intake and outtake===");
            telemetry.addData("Intake Power:", "%f", intake.getPower());
            telemetry.addData("Outtake Power:","%f", outtake.getPower());

            telemetry.update();
        }

    }

    private void updateDrivetrain(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * rl;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);

    }

    private void updateAuxiliaryMotors(){

        transfer.setPower(0);
        // flip intake direction toggle
        if (gamepad1.right_bumper && !bumper)
            flip = -flip;  // Flip once per press
        bumper = gamepad1.right_bumper;

        double intakePower = gamepad1.x ? flip : 0.0;
        double outtakePower = gamepad1.right_trigger > 0.6 ? 1 : 0;

        if (gamepad1.a && !aPressed) {
            servopos = !servopos;
            aPressed = true;
        }

        if (!gamepad1.a) aPressed = false;

        if(servopos){
            b1.setPosition(B1C);
        }else
            b1.setPosition(B1U);



        if (gamepad1.dpad_down && !dp) {
            adjust -= 0.05;
        }
        if (gamepad1.dpad_up && !du){
            adjust += 0.05;
        }

        // Timed transfer control
        /*if (gamepad1.y && !transferActive) {
            // start the transfer for a brief time
            transferActive = true;
            transferTimer.reset();
            transfer.setPower(1);
        }

        if (transferActive) {
            // stop after duration
            if (transferTimer.seconds() > transferDuration) {
                transfer.setPower(0);
                transferActive = false;
            }
        }*/

        // Manual reverse (left bumper)
        if (gamepad1.left_bumper) {
            transfer.setPower(-1);
        }

        if(gamepad1.y){
            transfer.setPower(1);
        }

        intake.setPower(intakePower);
        outtake.setPower(outtakePower*adjust);

        dp = gamepad1.dpad_down;
        du = gamepad1.dpad_up;

        telemetry.addData("Outtake Power: ", adjust);
        telemetry.addData("up: ", runningUp);
        telemetry.addData("down: ", runningDown);
    }


}








