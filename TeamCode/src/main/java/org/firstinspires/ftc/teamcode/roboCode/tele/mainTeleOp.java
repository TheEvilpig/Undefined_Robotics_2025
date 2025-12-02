package org.firstinspires.ftc.teamcode.roboCode.tele;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;


@TeleOp(name="Main_TeleOp", group="Linear OpMode")
public class mainTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime;
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive,
            intake, outtake1, outtake2, transfer;
    private Servo b1, b2;
    private boolean aPressed = false;
    private boolean servopos = false;


    private Limelight3A limeLight;
    private IMU imu;
    private Follower follower;

    private boolean rBumper = false;
    private boolean lBumper = false;
    private double rl = .8;

    double B2U = 0.19;
    double B2C = 0;

    double B1U = 0.83;
    double B1C = 1;
    private double adjust = 1;
    private double rPos = 0;

    private double lPos =0;
    private double flip =1;

    Orientation angles;
    @Override
    public void runOpMode() {

        //<editor-fold desc="INIT">
        //follower = Constants.createFollower(hardwareMap);

        ElapsedTime runtime = new ElapsedTime();
        //chassis
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rb");
        //shooting system
        intake = hardwareMap.get(DcMotor.class,"Intake");
        //outtake 1 is in same orientation as previous outtake motor
        outtake1 = hardwareMap.get(DcMotor.class,"Outtake1");
        outtake2 = hardwareMap.get(DcMotor.class,"Outtake2");
        transfer = hardwareMap.get(DcMotor.class,"Transfer");
        //braking system
        b2 = hardwareMap.get(Servo.class, "ls");
        b1 = hardwareMap.get(Servo.class, "rs");
        //April Tags
        limeLight = hardwareMap.get(Limelight3A.class,"Limelight");
        //imu
        imu = hardwareMap.get(IMU.class,"IMU");

        //directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        b1.setPosition(B1U);
        b2.setPosition(B2U);
        limeLight.setPollRateHz(100);
        limeLight.pipelineSwitch(1);
        limeLight.start();


        telemetry.addData("Status", "Initialized");

        //</editor-fold>

        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updateDrivetrain();

            updateAuxiliaryMotors();

            scanAprilTags();
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("===Testing intake and outtake===");
            telemetry.addData("Intake Power:", "%f", intake.getPower());
            telemetry.addData("Outtake Power:","%f", (outtake1.getPower()+outtake2.getPower())/2);

            scanAprilTags();

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
/*
        frontLeftPower = gamepad1.dpad_up ? 1.0:0;
        frontRightPower = gamepad1.dpad_right ? 1.0:0;
        backRightPower = gamepad1.dpad_down ? 1.0:0;
        backLeftPower = gamepad1.dpad_left ? 1.0:0;
*/


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

        if (gamepad1.a && !aPressed) {
            servopos = !servopos;
            aPressed = true;
        }

        if (!gamepad1.a) aPressed = false;

        if(servopos){
            b1.setPosition(B1C);
            b2.setPosition(B2C);
        }else {
            b1.setPosition(B1U);
            b2.setPosition(B2U);
        }


        if(gamepad1.dpad_down)
            adjust-=.05;
        if(gamepad1.dpad_up)
            adjust+=.05;


        //intake.setPower(intakePower);
        outtake1.setPower(outtakePower);
        outtake2.setPower(outtakePower);
        //transfer.setPower(transferPower);

    }

    private void scanAprilTags(){
        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));
        //latest limelight result
        LLResult result = limeLight.getLatestResult();
        if(result!=null && result.isValid()){
            Pose3D pose = result.getBotpose_MT2();
            telemetry.addData("Target X (Degrees)",result.getTx());
            telemetry.addData("Target Y (Degrees)",result.getTy());
            telemetry.addData("Botpose",pose.toString());

        }
        else
            telemetry.addLine("No targets detected from Limelight.");



    }

}


/*        LLResult result = limeLight.getLatestResult();
        if(result!=null&&result.isValid()){
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if(!fiducials.isEmpty()){
                LLResultTypes.FiducialResult tag = fiducials.get(0); //1st detected tag
                int id = tag.getFiducialId();
                Pose3D pose = tag.getRobotPoseTargetSpace();
                telemetry.addData("Tag ID: ", id);
                telemetry.addData("X (Meters): ",pose.getPosition().x);
                telemetry.addData("Y (Meters): ",pose.getPosition().y);
                telemetry.addData("Z (Meters): ", pose.getPosition().z);
                telemetry.addData("Yaw (Degrees): ", Math.toDegrees(pose.getOrientation().getYaw()));
                telemetry.addData("Pitch (Degrees): ", Math.toDegrees(pose.getOrientation().getPitch()));
                telemetry.addData("Roll (Degrees): ", Math.toDegrees(pose.getOrientation().getRoll()));
            }
            else
                telemetry.addLine("No tags detected.");
        }
        else
            telemetry.addLine("No valid result.");

 */