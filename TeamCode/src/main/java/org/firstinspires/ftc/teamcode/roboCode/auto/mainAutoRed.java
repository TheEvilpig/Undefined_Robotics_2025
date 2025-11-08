package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Main Autonomous Red",group = "Autonomous")
public class mainAutoRed extends LinearOpMode {
    DcMotor frontLeftDrive;
    DcMotor backLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backRightDrive;
    DcMotor intake;
    DcMotor outtake;
    DcMotor transfer;
    Follower follower;
    GoBildaPinpointDriver odoComputer;

    private Timer pathTimer;
    private int pathState;
    private int shootingState = 0;
    private int ballsShot = 0;

    // Power variables for intake, outtake, and transfer
    private double intakePower = 0.0;
    private double outtakePower = 0.0;
    private double transferPower = 0.0;
    private double targetSpeed = 0.0;

    // Define poses
    private final Pose startPose = new Pose(84, 12, Math.toRadians(0));
    private final Pose targetPose = new Pose(84, 36, Math.toRadians(45));

    // Define path
    private PathChain pathToTarget;

    @Override
    public void runOpMode(){
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rb");

        intake = hardwareMap.get(DcMotor.class,"Intake");
        outtake = hardwareMap.get(DcMotor.class,"Outtake");
        transfer = hardwareMap.get(DcMotor.class,"Transfer");

        //Odo. computer
        odoComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odoComputer");
        odoComputer.initialize();
        odoComputer.resetPosAndIMU();

        telemetry.addLine("===motors and odometry initialized===");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize timer
        pathTimer = new Timer();
        pathState = 0;

        // Build paths
        buildPaths();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Position", "X: %.1f, Y: %.1f", startPose.getX(), startPose.getY());
        telemetry.addData("Target Position", "X: 84.0, Y: 36.0");
        telemetry.update();

        waitForStart();

        // Start the path timer and set initial state
        pathTimer.resetTimer();
        setPathState(0);

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()){
            // Update follower
            follower.update();

            // Update autonomous path state
            autonomousPathUpdate();

            // Display telemetry
            telemetry.addData("Path State", pathState);
            telemetry.addData("Current X", "%.2f", follower.getPose().getX());
            telemetry.addData("Current Y", "%.2f", follower.getPose().getY());
            telemetry.addData("Current Heading", "%.2f degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Target X", "84.00");
            telemetry.addData("Target Y", "36.00");
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.update();
        }

        // Stop all motors when OpMode ends
        powerAllMotors(0);
    }

    public void buildPaths() {
        // Build path to target coordinates (84, 36)
        pathToTarget = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following path to target
                follower.followPath(pathToTarget);
                setPathState(1);
                break;
            case 1:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    // Path complete - stop all motors
                    powerAllMotors(0);
                    telemetry.addData("Status", "Path Complete!");
                    telemetry.addData("Final Position", "X: %.2f, Y: %.2f",
                            follower.getPose().getX(), follower.getPose().getY());
                    setPathState(2);
                }
                break;
            case 2:
                // Shoot preloaded balls - shoot 3 balls one at a time
                if (ballsShot < 3) {
                    shootBallUpdate();
                } else {
                    // All balls shot, move to next state
                    transfer.setPower(0);
                    outtake.setPower(0);
                    setPathState(3);
                }
                break;
            case 3:
                // Finished
                break;
        }
    }

    public void shootBallUpdate() {
        switch (shootingState) {
            case 0:
                // Start transfer
                transfer.setPower(transferPower);
                pathTimer.resetTimer();
                shootingState = 1;
                break;
            case 1:
                // Wait 0.2 seconds for transfer
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    shootingState = 2;
                    transfer.setPower(0);
                }
                break;
            case 2:
                // Shoot the ball
                outtakePower = targetSpeed;
                outtake.setPower(outtakePower);
                pathTimer.resetTimer();
                shootingState = 3;
                break;
            case 3:
                // Wait 0.5 seconds for ball to shoot
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    // Stop motors
                    transfer.setPower(0);
                    outtake.setPower(0);
                    ballsShot++;
                    shootingState = 0;  // Reset for next ball
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        // Reset shooting state when entering shooting phase
        if (pState == 2) {
            shootingState = 0;
            ballsShot = 0;
        }
    }

    //stopping all motors
    public void powerAllMotors(double power){
        frontRightDrive.setPower(power);
        frontLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        intake.setPower(power);
        outtake.setPower(power);
        transfer.setPower(power);
    }

}
