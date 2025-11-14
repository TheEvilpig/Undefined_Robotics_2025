package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;


@TeleOp(name="Main_TeleOp", group="Linear OpMode")
public class mainTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime;
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive,
            intake, outtake, transfer;

    private Servo rBrake, lBrake;
    private Limelight3A limeLight;
    private Follower follower;

    private boolean rBumper = false;
    private boolean lBumper = false;
    private double rl = .8;

    private double adjust = 1;
    private double rPos = 0;

    private double lPos =0;
    private double flip =1;

    Orientation angles;
    @Override
    public void runOpMode() {

        //<editor-fold desc="INIT">
        follower = Constants.createFollower(hardwareMap);

        ElapsedTime runtime = new ElapsedTime();
        //chassis
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rb");
        //shooting system
        intake = hardwareMap.get(DcMotor.class,"Intake");
        outtake = hardwareMap.get(DcMotor.class,"Outtake");
        transfer = hardwareMap.get(DcMotor.class,"Transfer");
        //braking system
        rBrake = hardwareMap.get(Servo.class,"rb");
        lBrake = hardwareMap.get(Servo.class,"lb");
        //April Tags
        limeLight = hardwareMap.get(Limelight3A.class,"Limelight");

        //directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        limeLight.start();

        telemetry.addData("Status", "Initialized");

        //</editor-fold>

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

            scanAprilTags();

            telemetry.update();
        }

    }

    private void updateDrivetrain(){
        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
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
        if (gamepad1.right_bumper&&!rBumper)
            flip = -flip;  // Flip once per press
        rBumper = gamepad1.right_bumper;

        double intakePower = gamepad1.x ? flip :0.0;
        double transferPower = gamepad1.y ? flip :0;
        double outtakePower = gamepad1.b ? adjust:0;
        double rBrakePosition = 0;

        if(gamepad1.left_bumper&&!lBumper){
            //activate braking
            rBrake.setPosition(rBrakePosition);
        }
        lBumper = gamepad1.left_bumper;
        if(gamepad1.dpad_down)
            adjust-=.05;
        if(gamepad1.dpad_up)
            adjust+=.05;


        intake.setPower(intakePower);
        outtake.setPower(outtakePower);
        transfer.setPower(transferPower);

    }
    private void scanAprilTags(){
        LLResult result = limeLight.getLatestResult();
        if(result!=null&&result.isValid()){
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if(!fiducials.isEmpty()){
                LLResultTypes.FiducialResult tag = fiducials.get(0); //1st detected tag
                int id = tag.getFiducialId();
                Pose3D pose = tag.getRobotPoseTargetSpace();
                telemetry.addData("Tag ID: ", id);
                telemetry.addData("X (Meters): ",pose.getX());
                telemetry.addData("Y (Meters): ",pose.getY());
                telemetry.addData("Z (Meters): ", pose.getZ());
                telemetry.addData("Yaw (Degrees): ", Math.toDegrees(pose.getRotation().getYaw()));
            }
            else
                telemetry.addLine("No tags detected.");
        }
        else
            telemetry.addLine("No valid result.");
    }
}
