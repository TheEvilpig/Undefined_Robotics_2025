package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Simple Auto Red", group = "Autonomous")
public class simpleAutoRed extends LinearOpMode {

    // Declare motors
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private DcMotor intake;
    private DcMotor outtake;
    private DcMotor transfer;

    // Timer for time-based movements
    private ElapsedTime runtime = new ElapsedTime();

    // Constants for encoder-based movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: goBILDA 312 RPM Yellow Jacket
    private static final double DRIVE_GEAR_REDUCTION = 1.0;       // No External Gearing
    private static final double WHEEL_DIAMETER_INCHES = 4.0;      // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rb");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run using encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ============ AUTONOMOUS SEQUENCE ============

        encoderDrive(0.5, 5, 5, 3.0);

        sleep(1000);

        turnRight(0.4, .7);

        shootBalls(1);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Drive straight using encoders
     *
     * @param speed       Speed (0 to 1.0)
     * @param leftInches  Distance for left wheels
     * @param rightInches Distance for right wheels
     * @param timeoutS    Timeout in seconds
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            // Calculate new target positions
            newLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            // Set target positions
            frontLeftDrive.setTargetPosition(newLeftTarget);
            backLeftDrive.setTargetPosition(newLeftTarget);
            frontRightDrive.setTargetPosition(newRightTarget);
            backRightDrive.setTargetPosition(newRightTarget);

            // Turn on RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset runtime and start motion
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            // Keep looping while we are still active, and there is time left, and both motors are running
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftDrive.isBusy() && frontRightDrive.isBusy())) {

                // Display info
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            stopDriveMotors();

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250); // Pause for a moment
        }
    }


    /**
     * Turn right (clockwise)
     *
     * @param speed       Speed (0 to 1.0)
     * @param timeSeconds Time to turn in seconds
     */
    public void turnRight(double speed, double timeSeconds) {
        runtime.reset();

        // Turn right: left motors forward, right motors backward
        frontLeftDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        frontRightDrive.setPower(-speed);
        backRightDrive.setPower(-speed);

        while (opModeIsActive() && (runtime.seconds() < timeSeconds)) {
            telemetry.addData("Turning", "Right");
            telemetry.update();
        }

        stopDriveMotors();
        sleep(250);
    }

    /**
     * Turn left (counter-clockwise)
     *
     * @param speed       Speed (0 to 1.0)
     * @param timeSeconds Time to turn in seconds
     */
    public void turnLeft(double speed, double timeSeconds) {
        runtime.reset();

        // Turn left: left motors backward, right motors forward
        frontLeftDrive.setPower(-speed);
        backLeftDrive.setPower(-speed);
        frontRightDrive.setPower(speed);
        backRightDrive.setPower(speed);

        while (opModeIsActive() && (runtime.seconds() < timeSeconds)) {
            telemetry.addData("Turning", "Left");
            telemetry.update();
        }

        stopDriveMotors();
        sleep(250);
    }

    /**
     * Shoot specified number of balls
     *
     * @param numBalls Number of balls to shoot
     */
    public void shootBalls(int numBalls) {
        for (int i = 0; i < numBalls; i++) {
            telemetry.addData("Shooting ball", "%d of %d", i + 1, numBalls);
            telemetry.update();

            outtake.setPower(0.8);
            sleep(1000);  // Wait 1 second
            // Transfer ball
            transfer.setPower(0.5);
            sleep(200);  // Wait 0.2 seconds
            transfer.setPower(0.0);
            sleep(500);  // Wait 0.5 seconds
            // Stop motors
            transfer.setPower(0);
            outtake.setPower(0);
            sleep(100);  // Short pause between balls
        }
    }

    /**
     * Stop all drive motors
     */
    public void stopDriveMotors() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /**
     * Stop all motors including mechanisms
     */
    public void stopAllMotors() {
        stopDriveMotors();
        intake.setPower(0);
        outtake.setPower(0);
        transfer.setPower(0);
    }
}
