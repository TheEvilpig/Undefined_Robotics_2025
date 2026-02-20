/*
package org.firstinspires.ftc.teamcode.roboCode.tele;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

    private double fieldHeadingOffset = MatchState.finalAutoHeadingDeg; // radians


    private DcMotorSystem shooter;

    private NormalizedColorSensor color;

    private double speed = 280;

    private final TriggerButton reverseTrigger =
            new TriggerButton(() -> gamepad1.left_trigger, 0.8);

    boolean thr = false;

    //Servos
    double vInc = 0;

    private Servo hold;

    private double aTError = 0;

    private boolean holding = true;

    // Shooting system
    private boolean shooterActive = false;

    private boolean isRed = false;

    private final ToggleButton isFast =
            new ToggleButton(() -> gamepad1.y);



    //limelight
    private Limelight3A limeLight;
    private LLResult latestResult = null;
    private double distance = 140;

    //needed to pair with limelight
    private IMU imu;

    //private final ToggleButton allOn = new ToggleButton(() -> gamepad1.x);
    private boolean allOn = false;
    private final TriggerButton shootTrigger =
            new TriggerButton(() -> gamepad1.right_trigger, 0.8);


    private boolean intaking = false;

    private final double rl = .8;

    // Turning PID
    private double lastError = 0;
    private double lastTime = 0;

    //private ArrayList<Artifact> artif;

    @Override

    public void runOpMode() {
        teleInit();

        waitForStart();

        setFieldCentricHeading(MatchState.finalAutoHeadingDeg);

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
/*
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
        transfer = hardwareMap.get(DcMotorEx.class, HConst.TRANSFER2);

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
/*

        //Color Sensor
        color = hardwareMap.get(NormalizedColorSensor.class, HConst.COLOR);


        //April Tags
        limeLight = hardwareMap.get(Limelight3A.class, "Limelight");

        //imu
        imu = hardwareMap.get(IMU.class, "IMU");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);


        //directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(HConst.INTAKE_DIR);
        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer.setDirection(HConst.TRANSFER1_DIR);

        shooter.addFollower(outtake1);
        shooter.setPID(
                0.00845,  // kP
                0.00015,  // kI
                0.00175,  // kF
                0.1358     // kStatic
        );
        shooter.setTargetVelocity(285);
        shooter.enable();

        limeLight.setPollRateHz(100);
        limeLight.pipelineSwitch(1);
        limeLight.start();

        telemetry.addData("Status", "Initialized");
    }

    /**
     * Drives the robot chassis motors
     */
/*
    private void updateDrivetrain() {


        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;

        if (driveTrainMode == HConst.DriveTrainMode.AUTO_TARGET_GOAL) {

            double error = latestResult.getTx();
            aTError = error;

            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastTime;
            double derivative = deltaTime > 0 ? (error - lastError) / deltaTime : 0;

            double p = 0.02;
            double kStatic = 0.016;

            double turnPower = p * error;

            if (Math.abs(error) > 0.5) {
                turnPower += Math.signum(error) * kStatic;
            }

            turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

            // Smooth slow-down near target
            if (Math.abs(error) < 2) {
                turnPower *= 0.85;
            }
            if (Math.abs(error) < 1.45) {
                turnPower *= 0.65;
            }

            if (Math.abs(error) < 0.8) {
                turnPower = 0;
            }


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
            double y = applyDeadzone(-gamepad1.left_stick_y, 0.08) * (isFast.isToggled() ? 1 : 0.7);
            double x = applyDeadzone(gamepad1.left_stick_x, 0.08) * 1.1 * (isFast.isToggled() ? 1 : 0.7);
            double rx = applyDeadzone(gamepad1.right_stick_x, 0.08) * rl * (isFast.isToggled() ? 1 : 0.5);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

        } else if (driveTrainMode == HConst.DriveTrainMode.GAMEPAD_FIELD_CENTRIC){
            double y = applyDeadzone(-gamepad1.left_stick_y, 0.08) * (isFast.isToggled() ? 1 : 0.7);
            double x = applyDeadzone(gamepad1.left_stick_x, 0.08) * 1.1 * (isFast.isToggled() ? 1 : 0.7);
            double rx = applyDeadzone(gamepad1.right_stick_x, 0.08) * rl * (isFast.isToggled() ? 1 : 0.5);

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = normalizeAngle(rawHeading + fieldHeadingOffset -
                    (isRed ? 90 : -90));
            //double botHeading = rawHeading;

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
/*
    private void updateAuxiliaryMotors() {
        double transferPower = 0;
        double intakePower = 0.2;

        if (allOn) {
            intakePower = 1;
            transferPower = .42;
        }

        if(intaking)
            intakePower = 0.5;


        //transferPower = shooterActive ? tpower : transferPower;

        if((shooterActive) && distance >= 0) {
            shooter.setTargetVelocity(shooter.computeTargetVelocityFromDistance(distance) + vInc);
        } else if(distance>=0){
            shooter.setTargetVelocity(shooter.computeTargetVelocityFromDistance(distance) + vInc);
        } else {
            shooter.setTargetVelocity(280);
        }


        if(intaking)
            transferPower = 0.8;

        if(reverseTrigger.isPressed()){
            shooter.setTargetVelocity(-100);
            transferPower = -1;
            intakePower = -1;
        }

        if (gamepad1.dpad_down) {
            vInc = vInc - 1;
        }

        if (gamepad1.dpad_up) {
            vInc = vInc + 1;
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
        if(driveTrainMode==HConst.DriveTrainMode.AUTO_TARGET_GOAL)
            telemetry.addData("Drive Mode:", "ALIGNMENT");

        telemetry.addData("Outtake Velocity:", shooter.getMeasuredVelocity());
        telemetry.addData("Outtake Target Velocity:", shooter.getTargetVelocity());
        //telemetry.addLine("===Color Detected===");
        //alpha = color.getNormalizedColors().alpha;

        telemetry.addData("dist: ", distance);
        telemetry.addData("---------------------------------------------------", "-");
        telemetry.addData("SIDE: ", isRed ? "RED" : "BLUE");
        telemetry.addData("---------------------------------------------------", "-");
        telemetry.addData("Velocity Increment: ", vInc);
        telemetry.addData("---------------------------------------------------", "-");
        telemetry.addData("Speed: ", isFast.isToggled() ? "FAST" : "SLOW");
        telemetry.addData("---------------------------------------------------", "-");
        /*if(latestResult != null && latestResult.isValid()) {
            telemetry.addData("Capture (ms)", latestResult.getCaptureLatency());
            telemetry.addData("Parse (ms)", latestResult.getParseLatency());
            telemetry.addData("Targeting (ms)", latestResult.getTargetingLatency());
        }*.\/

        /*
        telemetry.addData("shooter is toggled: ", shooterOn.isToggled());
        telemetry.addData("dist: ", distance);
        telemetry.addData("t velocity: ", shooter.computeTargetVelocityFromDistance(distance));
        telemetry.addData("enabled: ", shooter.isEnabled());
         */




    /**
     * Scans april tags on targets to help in robot allignment for accurate scoring trajectory.
     */
    /*
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
                distance = 140;
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
    /*
    private void scanGamepads(){
        //update toggleable buttons

        //gamepad1.dpadLeft
        //shooterOn.update();
        //gamepad1.dpadUp
        allOn = gamepad1.x;

        isFast.update();

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
            thr = false;
        } else {
            double t = runtime.seconds() - seqTime;
            double p = 0.12;
            double s = 0.55;

            shooterActive = true;
            intaking = true;
            //tpower =  0.1 + Math.pow(t, 1.3) * 0.1;

            if(t < 0.15){
                driveTrainMode = HConst.DriveTrainMode.STOP;
            } else{
                driveTrainMode = HConst.DriveTrainMode.AUTO_TARGET_GOAL;
            }

            boolean velocityReady =
                    Math.abs(shooter.getMeasuredVelocity() - shooter.getTargetVelocity()) <= 3.5;

            boolean angleReady = Math.abs(aTError) < 2;

            boolean shooterReady = velocityReady && angleReady;

            if (!thr) {

                if (t < 0.8) {
                    // Phase 1: never shoot
                    holding = true;

                } else if (t < 1.5) {
                    // Phase 2: shoot only if velocity good
                    holding = !shooterReady;

                    if (shooterReady) {
                        thr = true;
                    }

                } else {
                    // Phase 3: force shoot
                    holding = false;
                    thr = true;
                }
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
    /*
    private double getDistance(LLResult result) {

        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));

        //latest limelight result
        if (result != null && result.isValid())
            return (HConst.LLH2 - HConst.LLH1) / (Math.tan(Math.toRadians(HConst.LLANGL + result.getTy())));

        return 140;
    }

    public double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void setFieldCentricHeading(double desiredHeadingDegrees) {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double desiredHeading = Math.toRadians(desiredHeadingDegrees);

        fieldHeadingOffset = currentHeading - desiredHeading;
    }
    private double applyDeadzone(double value, double dz) {
        if (Math.abs(value) < dz) return 0;
        return value;
    }

}
/*

     */