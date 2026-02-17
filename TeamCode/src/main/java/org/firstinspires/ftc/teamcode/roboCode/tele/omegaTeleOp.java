package org.firstinspires.ftc.teamcode.roboCode.tele;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.DcMotorSystem;
import org.firstinspires.ftc.teamcode.util.HConst;
import org.firstinspires.ftc.teamcode.util.*;

import java.util.ArrayList;
/*
@TeleOp(name="OMEGA OP", group="Linear OpMode")

public class omegaTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();


    // Declare OpMode members for each of the 4 motors.
    //DcMotorEx is basically like DcMotor but more advanced with more methods and capabilities
    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive,
            intake, outtake1, outtake2, transfer;
    private HConst.DriveTrainMode driveTrainMode = HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC;
    private HConst.DriveTrainMode defaultMode = HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC;

    private final Pose2D BLUE_LEFT_START,BLUE_RIGHT_START,RED_LEFT_START,RED_RIGHT_START;
    //sensors
    private Limelight3A ll;
    private LLResult latestResult;
    private IMU imu;

    //Buttons
    TriggerButton cycle = new TriggerButton(()->gamepad1.right_trigger,.1);
    ToggleButton fire = new ToggleButton(()->gamepad1.a);
    @Override

    public void runOpMode() {
        teleInit();

        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            scanAprilTags();
            scanGamepads();
            updateDrivetrain();
            updateAuxiliaryMotors();
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }

    }

    /**
     * intitialize, hardwaremap
     */
  /*  private void teleInit() {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, HConst.LEFT_FRONT);
        backLeftDrive = hardwareMap.get(DcMotorEx.class, HConst.LEFT_BACK);
        frontRightDrive = hardwareMap.get(DcMotorEx.class, HConst.RIGHT_FRONT);
        backRightDrive = hardwareMap.get(DcMotorEx.class, HConst.RIGHT_BACK);

        //Cycling motors
        intake = hardwareMap.get(DcMotorEx.class, HConst.INTAKE);
        //outtake 1 is in same orientation as previous outtake motor
        outtake1 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);
        transfer = hardwareMap.get(DcMotorEx.class, HConst.TRANSFER);

        //April Tags
        ll = hardwareMap.get(Limelight3A.class, "Limelight");

        //IMU
        imu = hardwareMap.get(IMU.class, "IMU");

        //Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(HConst.INTAKE_DIR);
        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer.setDirection(HConst.TRANSFER_DIR);

        ll.setPollRateHz(10);
        ll.pipelineSwitch(1);
        ll.start();
        telemetry.addLine("Initialized");

        BLUE_LEFT_START = new Pose2D(12,60,0);
        BLUE_RIGHT_START = new Pose2D();
    }

    /**
     * Drives the robot chassis motors
     */
    /*private void updateDrivetrain() {

        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        if (driveTrainMode == HConst.DriveTrainMode.AUTO_TARGET_GOAL) {

            double error = latestResult.getTx();
            double turnPower = .5*error/100.;

            turnPower = Math.max(-.5, Math.min(.5, turnPower));
            if (Math.abs(error) < 1.2)
                turnPower = 0;
            frontLeftPower = turnPower;
            backLeftPower = turnPower;
            frontRightPower = -turnPower;
            backRightPower = -turnPower;
        } else if (driveTrainMode == HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC){
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

        } else if (driveTrainMode == HConst.DriveTrainMode.GAMEPAD_FIELD_CENTRIC){
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;
        } else if (driveTrainMode == HConst.DriveTrainMode.STOP){
            frontLeftPower = 0;
            backLeftPower = 0;
            frontRightPower = 0;
            backRightPower = 0;
        }
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    /**
     * Drives the intake, transfer, outtake motors and braking system servos.
     */
/*private void updateAuxiliaryMotors() {
        double iPower, tPower, oPower = 0;
        if(cycle.isPressed()&&!fire.isToggled()){
            iPower=.5;
            tPower=.5;
        }
        else if(cycle.isPressed()&&fire.isToggled()){
            //outa7
        }
        else if(gamepad1.dpad_down)
            tPower=-.25;


    }

    }

    /**
     * Scans april tags on targets to help in robot allignment for accurate scoring trajectory.
     */
    /*private void scanAprilTags() {

        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));
        //latest limelight result
        LLResult result = limeLight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D pose = result.getBotpose_MT2();

            telemetry.addData("Target X (Degrees)", result.getTx());
            telemetry.addData("Target Y (Degrees)", result.getTy());
            telemetry.addData("Botpose", pose.toString());

            int id = result.getFiducialResults().get(0).getFiducialId();

            if (id == 21)
                seq = "gpp";
            if (id == 22)
                seq = "pgp";
            if (id == 23)
                seq = "ppg";

            if(id == (isRed ? 24 : 20))
                distance = getDistance(result) * 39.3701 + 7.5 ;
            else{
                distance = -1;
            }
            latestResult = result;

        } else {
            latestResult = null;

            telemetry.addLine("No targets detected from Limelight.");
        }
    }

    /**
     * Scans gamepad to update teleop controlls
     */
    /*private void scanGamepads(){
        //update toggleable buttons

        //gamepad1.dpadLeft
        //shooterOn.update();
        //gamepad1.dpadUp
        allOn = gamepad1.x;

        //intake / transfer
        //intaking = gamepad1.x;

        //drive mode
        driveTrainMode = defaultMode;

        if(gamepad1.left_bumper)
            defaultMode = HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC;
        else if(gamepad1.right_bumper)
            defaultMode = HConst.DriveTrainMode.GAMEPAD_FIELD_CENTRIC;

        if(!shootTrigger.isPressed()){
            seqTime = runtime.seconds();
            shooterActive = false;
            holding = true;
            intaking = false;
        } else {
            double t = runtime.seconds() - seqTime;
            double p = 0.12;
            double s = 0.55;

            shooterActive = true;
            intaking = true;
            //tpower =  0.1 + Math.pow(t, 1.3) * 0.1;

            if(t < 0.4){
                driveTrainMode = HConst.DriveTrainMode.STOP;
            } else{
                driveTrainMode = HConst.DriveTrainMode.AUTO_TARGET_GOAL;
            }

            if (t < 1.7) {
                holding = true;
            } else if (t < 1.5 + p + 10) {
                holding = false;
            } else if (t < 1.5 + p + s + 10) {
                holding = true;
            } else if (t < 1.5 + 2 * p + s) {
                holding = false;
            } else if (t < 1.5 + 2 * p + 2 * s) {
                holding = true;
            } else if (t < 1.5 + 3 * p + 2 * s) {
                holding = false;
            } else if (t < 1.5 + 3 * p + 3 * s) {
                holding = true;
            } else {
                holding = false;
            }

        }

        if(latestResult == null){
            driveTrainMode = defaultMode;
        }


        if(gamepad1.dpad_right)
            isRed = false;

        if(gamepad1.dpad_left)
            isRed = true;

    }

    /**
     * Calculates the distance the robot is from the target, using data from the limelight
     *
     * @return the horizontal displacement/distance from the robot to the target.
     */
    /*private double getDistance(LLResult result) {

        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));

        //latest limelight result
        if (result != null && result.isValid())
            return (HConst.LLH2 - HConst.LLH1) / (Math.tan(Math.toRadians(HConst.LLANGL + result.getTy())));

        return -1;
    }


}*/
