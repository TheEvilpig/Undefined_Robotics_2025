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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.DcMotorSystem;
import org.firstinspires.ftc.teamcode.util.HConst;
import org.firstinspires.ftc.teamcode.util.*;

import java.util.ArrayList;

@TeleOp(name="Main_TeleOp", group="Linear OpMode")

public class mainTeleOp extends LinearOpMode {

    public static String seq;
    private final ElapsedTime runtime = new ElapsedTime();

    private double seqTime = 0;

    // Declare OpMode members for each of the 4 motors.
    //DcMotorEx is basically like DcMotor but more advanced with more methods and capabilities
    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive,
            intake, outtake1, outtake2, transfer;

    private HConst.DriveTrainMode driveTrainMode = HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC;
    private HConst.DriveTrainMode defaultMode = HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC;

    private DcMotorSystem shooter;

    private NormalizedColorSensor color;

    //Servos

    private Servo hold;

    private boolean holding = true;

    // Shooting system
    private boolean shooterActive = false;

    private final ElapsedTime shootTimer = new ElapsedTime();

    private boolean isRed = false;


    //limelight
    private Limelight3A limeLight;
    private LLResult latestResult = null;
    private double distance = -1;

    //needed to pair with limelight
    private IMU imu;

    private final ToggleButton shooterOn = new ToggleButton(() -> gamepad1.dpad_left);
    private final ToggleButton allOn = new ToggleButton(() -> gamepad1.dpad_up);
    private final TriggerButton shootTrigger =
            new TriggerButton(() -> gamepad1.right_trigger, 0.5);


    private boolean intaking = false;

    private final double rl = .8;
    private double alpha = 0;

    // Turning PID
    private double lastError = 0;
    private double lastTime = 0;

    //private ArrayList<Artifact> artif;

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
    private void teleInit() {
        //artif = new ArrayList<Artifact>();
        //carolPos = 0;
        //chassis
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, HConst.LEFT_FRONT);
        backLeftDrive = hardwareMap.get(DcMotorEx.class, HConst.LEFT_BACK);
        frontRightDrive = hardwareMap.get(DcMotorEx.class, HConst.RIGHT_FRONT);
        backRightDrive = hardwareMap.get(DcMotorEx.class, HConst.RIGHT_BACK);

        //shooting system
        intake = hardwareMap.get(DcMotorEx.class, HConst.INTAKE);

        //outtake 1 is in same orientation as previous outtake motor
        outtake1 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);
        transfer = hardwareMap.get(DcMotorEx.class, HConst.TRANSFER);

        //shooter system
        shooter = new DcMotorSystem(
                outtake2,
                28,     // ticks per rev
                0.01, // velocity update interval
                telemetry
        );

        //hold
        hold = hardwareMap.get(Servo.class,HConst.HOLD);

        //sorting system
        /*
        carol = hardwareMap.get(Servo.class,"Sort");

         */

        //Color Sensor
        color = hardwareMap.get(NormalizedColorSensor.class, HConst.COLOR);


        //April Tags
        limeLight = hardwareMap.get(Limelight3A.class, "Limelight");

        //imu
        imu = hardwareMap.get(IMU.class, "IMU");

        //directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(HConst.INTAKE_DIR);
        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer.setDirection(HConst.TRANSFER_DIR);

        shooter.addFollower(outtake1);
        shooter.setPID(
                0.002,  // kP
                0.003,  // kI
                0.00155,  // kF
                0.1528     // kStatic
        );
        shooter.setTargetVelocity(0);
        shooter.enable();

        limeLight.setPollRateHz(100);
        limeLight.pipelineSwitch(1);
        limeLight.start();

        telemetry.addData("Status", "Initialized");
    }

    /**
     * Drives the robot chassis motors
     */
    private void updateDrivetrain() {

        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;

        if (driveTrainMode == HConst.DriveTrainMode.AUTO_TARGET_GOAL) {

            double error = latestResult.getTx();
            double k = 0.12;
            double p = 0.012;
            double d = 0.004;

            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastTime;
            double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;

            double turnPower = p * error + d * derivative + Math.signum(error) * k;

            turnPower = Math.max(-.5, Math.min(.5, turnPower));

            if (Math.abs(error) < 0.55)
                turnPower = 0;

            telemetry.addData("Turn Power:", turnPower);
            telemetry.addData("Error:", error);
            telemetry.addData("PID Derivative:", derivative);
            telemetry.addData("PID Delta Time:", deltaTime);

            frontLeftPower = turnPower;
            backLeftPower = turnPower;
            frontRightPower = -turnPower;
            backRightPower = -turnPower;

            lastError = error;
            lastTime = currentTime;

        } else if (driveTrainMode == HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC){
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * rl;

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

            rotX = rotX * 1.1;  // Counteract imperfect strafing

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
    private void updateAuxiliaryMotors() {

        double intakePower = intaking ? 1 : 0;
        double transferPower = intaking ? 0.75 : 0;

        if((shooterActive) && distance >= 0) {
            shooter.setTargetVelocity(shooter.computeTargetVelocityFromDistance(distance));
        } else {
            shooter.setTargetVelocity(0);
        }

        if (allOn.isToggled()) {
            intakePower = .9;
            transferPower = .9;
        }

        if (gamepad1.dpad_down) {
            transferPower = -.22;
        }

        //servo stopper
        double holdPos = holding ? HConst.HOLD_ACTIVE : HConst.HOLD_INACTIVE;

        hold.setPosition(holdPos);
        transfer.setPower(transferPower);
        intake.setPower(intakePower);
        shooter.update();

        telemetry.addLine("===Testing intake and outtake===");
        telemetry.addData("Outtake Power:", "%f", (outtake1.getPower() + outtake2.getPower()) / 2);
        if(driveTrainMode == HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC)
            telemetry.addData("Drive Mode:", "robot centric");
        if(driveTrainMode == HConst.DriveTrainMode.GAMEPAD_FIELD_CENTRIC)
            telemetry.addData("Drive Mode:", "field centric");

        telemetry.addData("Outtake Velocity:", shooter.getMeasuredVelocity());
        telemetry.addData("Outtake Target Velocity:", shooter.getTargetVelocity());
        telemetry.addLine("===Color Detected===");
        alpha = color.getNormalizedColors().alpha;
        telemetry.addData("---------------------------------------------------", "-");
        telemetry.addData("SIDE: ", isRed ? "RED" : "BLUE");
        telemetry.addData("---------------------------------------------------", "-");
        telemetry.addData("dist: ", distance);
        /*
        telemetry.addData("shooter is toggled: ", shooterOn.isToggled());
        telemetry.addData("dist: ", distance);
        telemetry.addData("t velocity: ", shooter.computeTargetVelocityFromDistance(distance));
        telemetry.addData("enabled: ", shooter.isEnabled());
         */


    }

    /**
     * Scans april tags on targets to help in robot allignment for accurate scoring trajectory.
     */
    private void scanAprilTags() {

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
    private void scanGamepads(){
        //update toggleable buttons

        //gamepad1.dpadLeft
        shooterOn.update();
        //gamepad1.dpadUp
        allOn.update();

        //intake / transfer
        intaking = gamepad1.x;

        //auto shooter
        shooterActive = shooterOn.isToggled();

        //drive mode
        driveTrainMode = defaultMode;

        if(!shootTrigger.isPressed()){
            seqTime = runtime.seconds();
            holding = true;
        } else {
            double t = runtime.seconds() - seqTime;
            double p = 0.12;
            double s = 0.55;

            shooterActive = true;
            intaking = true;

            if(t < 0.4){
                driveTrainMode = HConst.DriveTrainMode.STOP;
            } else{
                driveTrainMode = HConst.DriveTrainMode.AUTO_TARGET_GOAL;
            }

            if (t < 1.7) {
                holding = true;
            } else if (t < 1.5 + p) {
                holding = false;
            } else if (t < 1.5 + p + s) {
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

        if(gamepad1.left_bumper)
            defaultMode = HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC;
        else if(gamepad1.right_bumper)
            defaultMode = HConst.DriveTrainMode.GAMEPAD_FIELD_CENTRIC;

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
    private double getDistance(LLResult result) {

        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));

        //latest limelight result
        if (result != null && result.isValid())
            return (HConst.LLH2 - HConst.LLH1) / (Math.tan(Math.toRadians(HConst.LLANGL + result.getTy())));

        return -1;
    }

    /*
    private void fire() {
        char[] order = seq.toCharArray();
        for (int i = 0; i < 3;i++) {
            switch (order[i]) {
                case 'p':
            }
        }
    }

     */

    /*
    private void launch(String color) {
        Artifact.currentDisplacement = carol.getPosition();
        int index = 0;
        for (Artifact se : artif) {
            if (se.getColor().equals(color)) {
                index = artif.indexOf(se);
            }
        }

    }

     */
}
