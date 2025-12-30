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
    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members for each of the 4 motors.
    //DcMotorEx is basically like DcMotor but more advanced with more methods and capabilities
    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive,
            intake, outtake1, outtake2, transfer;

    private HConst.DriveTrainMode driveTrainMode = HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC;

    private DcMotorSystem shooter;

    private NormalizedColorSensor color;

    //Servos
    private Servo hold;


    //limelight
    private Limelight3A limeLight;
    private LLResult latestResult = null;
    private double distance = -1;

    //needed to pair with limelight
    private IMU imu;

    private ToggleButton shooterOn = new ToggleButton(() -> gamepad1.b);
    private ToggleButton allOn = new ToggleButton(() -> gamepad1.dpad_up);

    private boolean intaking = false;

    private double rl = .8;
    private double alpha = 0;

    //private ArrayList<Artifact> artif;

    @Override

    public void runOpMode() {
        teleInit();

        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updateDrivetrain();

            updateAuxiliaryMotors();

            scanAprilTags();

            scanGamepads();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }

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
            double k = 0.13;
            double p = 0.006;

            double turnPower = p * error + Math.signum(error) * k;

            turnPower = Math.max(-.5, Math.min(.5, turnPower));

            if (Math.abs(error) < 0.5)
                turnPower = 0;

            telemetry.addData("Turn Power:", turnPower);
            telemetry.addData("Error:", error);

            frontLeftPower = turnPower;
            backLeftPower = turnPower;
            frontRightPower = -turnPower;
            backRightPower = -turnPower;

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
        double transferPower = intaking ? 1 : 0;

        if(shooterOn.isToggled() && distance >= 0) {
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
        double holdPos = shooterOn.isToggled() ? HConst.HOLD_INACTIVE : HConst.HOLD_ACTIVE;

        hold.setPosition(holdPos);
        transfer.setPower(transferPower);
        intake.setPower(intakePower);
        shooter.update();

        telemetry.addLine("===Testing intake and outtake===");
        telemetry.addData("Outtake Power:", "%f", (outtake1.getPower() + outtake2.getPower()) / 2);
        telemetry.addData("Outtake1 Velocity:", outtake1.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Outtake2 Velocity:", outtake2.getVelocity(AngleUnit.RADIANS));
        telemetry.addLine("===Color Detected===");
        alpha = color.getNormalizedColors().alpha;
        telemetry.addData("Red:", color.getNormalizedColors().red / alpha);
        telemetry.addData("Green:", color.getNormalizedColors().green / alpha);
        telemetry.addData("Blue:", color.getNormalizedColors().blue / alpha);


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
                0.1, // velocity update interval
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
                0.015,  // kP
                0.0008,  // kI
                0.002,  // kF
                0.2     // kStatic
        );
        shooter.setTargetVelocity(0);

        limeLight.setPollRateHz(100);
        limeLight.pipelineSwitch(1);
        limeLight.start();

        telemetry.addData("Status", "Initialized");
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

            distance = getDistance(result);
            latestResult = result;

        } else {
            latestResult = null;

            telemetry.addLine("No targets detected from Limelight.");
        }
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

    private void scanGamepads(){
        //update toggleable buttons

        //gamepad1.b
        shooterOn.update();
        //gamepad1.dpadUp
        allOn.update();

        //drive mode
        if(gamepad1.left_bumper || latestResult == null)
            driveTrainMode = HConst.DriveTrainMode.GAMEPAD_ROBOT_CENTRIC;
        else if(gamepad1.right_bumper)
            driveTrainMode = HConst.DriveTrainMode.GAMEPAD_FIELD_CENTRIC;

        if(gamepad1.y && latestResult != null)
            driveTrainMode = HConst.DriveTrainMode.AUTO_TARGET_GOAL;

        intaking = gamepad1.x;


    }


    /*
    private void fire(){
        outtake1.setPower(-44);
        outtake2.setPower(-.44);
        transfer.setPower(-.44);
        sleep(800);
        outtake1.setPower(0);
        outtake2.setPower(0);
        transfer.setPower(0);
        sleep(100);
        outtake1.setPower(1.0);
        outtake2.setPower(1.0);
        sleep(2200);
        for(int i =0;i<3;i++){
            transfer.setPower(.5);
            intake.setPower(.5);
            sleep(50);
            transfer.setPower(0);
            intake.setPower(0);
            sleep(300);
        }
        outtake1.setPower(0);
        outtake2.setPower(0);
    }

     */

    /**
     * Calculates the perfect angular velocity needed for the outtake system to undergo to launch artifacts into the target.
     * This does NOT account for air resistance/drag
     *
     * @return velocity needed for robot to score into target.
     */
    /*
    private double calculateOuttakeAngularVelocity(LLResult result){
        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));

        double dist = getDistance(result);

        if(result!=null && result.isValid()) {
            if(dist == -1)
                return -1;
            //radius of outtake wheel
            double r = 1;
            //height of target, 40 inches in meters
            double h2 = 1.016;
            //height of outtake launch
            double hT = 0;
            //displacement of limelight from outtake, assuming they are both in the middle
            double xO = 0;
            //angle of artifact firing from outtake, in degrees.
            double fire = 0;
            //numerator of final equation
            double num = 9.8*(Math.pow(getDistance(result)+xO,2));
            double denom = 2*Math.cos(Math.toRadians(fire))*((hT-h2)+(dist+xO)*Math.tan(Math.toRadians(fire)));
            double linearVel = Math.sqrt(num/denom);

            return linearVel/r;
        }
        return -1;
    }

     */
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
