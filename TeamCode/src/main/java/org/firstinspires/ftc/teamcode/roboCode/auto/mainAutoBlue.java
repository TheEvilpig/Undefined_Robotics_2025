package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name="Main Autonomous Blue",group = "Autonomous")
public class mainAutoBlue extends LinearOpMode {
    DcMotorEx frontLeftDrive;
    DcMotorEx backLeftDrive;
    DcMotorEx frontRightDrive;
    DcMotorEx backRightDrive;
    DcMotorEx intake;
    DcMotorEx outtake;
    DcMotorEx outtake2;
    DcMotorEx transfer;
    Follower follower;
    GoBildaPinpointDriver odoComputer;

    private Timer pathTimer;
    private int pathState;
    private int shootingState = 0;
    private int ballsShot = 0;

    // Power variables for intake, outtake, and transfer
    private double intakePower = 1;
    private double outtakePower = 4.0;
    private double transferPower = 1;
    private double targetSpeed = 0.0;

    // Define poses
    private final Pose startPose = new Pose(84, 12, Math.toRadians(90));
    private final Pose targetPose = new Pose(84, 5, Math.toRadians(113));

    private final Pose targetPose1 = new Pose(84, -20, Math.toRadians(113));

    // Define path
    private PathChain pathToTarget;
    private PathChain pathToTarget1;

    @Override
    public void runOpMode(){
        intake = hardwareMap.get(DcMotorEx.class,"Intake");
        outtake = hardwareMap.get(DcMotorEx.class,"Outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");
        transfer = hardwareMap.get(DcMotorEx.class,"Transfer");

        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake2.setDirection(DcMotor.Direction.FORWARD);
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
    }

    public void buildPaths() {
        // Build path to target coordinates (84, 36)
        pathToTarget = follower.pathBuilder()
                .addPath(new BezierLine(startPose, targetPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading())
                .build();
        pathToTarget1 = follower.pathBuilder()
                .addPath(new BezierLine(targetPose, targetPose1))
                .setLinearHeadingInterpolation(targetPose.getHeading(), targetPose1.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                intake.setPower(0.3);
                // Start following path to target
                follower.followPath(pathToTarget);
                setPathState(1);
                break;
            case 1:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    // Path complete - stop all motors
                    telemetry.addData("Status", "Path Complete!");
                    telemetry.addData("Final Position", "X: %.2f, Y: %.2f",
                            follower.getPose().getX(), follower.getPose().getY());
                    pathTimer.resetTimer();
                    outtake.setVelocity(outtakePower, AngleUnit.RADIANS);
                    outtake2.setVelocity(outtakePower, AngleUnit.RADIANS);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() >= 3.5){
                    setPathState(3);
                    transfer.setPower(transferPower);
                    pathTimer.resetTimer();
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() >= 0.15){
                    setPathState(4);
                    transfer.setPower(0);
                }
                break;
            case 4:
                // Shoot preloaded balls - shoot 3 balls one at a time

                if (ballsShot < 3) {
                    shootBallUpdate();
                } else {
                    // All balls shot, move to next state
                    transfer.setPower(0);
                    outtake.setVelocity(0, AngleUnit.RADIANS);
                    outtake2.setVelocity(0, AngleUnit.RADIANS);
                    setPathState(5);
                    follower.followPath(pathToTarget1);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                break;
        }
    }

    public void shootBallUpdate() {
        switch (shootingState) {
            case 0:
                // Shoot the ball
                pathTimer.resetTimer();
                shootingState = 1;
                break;
            case 1:
                // Wait 0.2 seconds for transfer
                if (pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    // Stop motors
                    transfer.setPower(0);
                    ballsShot++;
                    shootingState = 2;  // Reset for next ball
                }
                break;
            case 2:
                // Start transfer
                transfer.setPower(transferPower);
                intake.setPower(intakePower);
                pathTimer.resetTimer();
                shootingState = 3;
                break;
            case 3:
                // Wait 0.5 seconds for ball to shoot
                if (pathTimer.getElapsedTimeSeconds() >= (ballsShot == 1 ? 0.25 : 6)) {
                    shootingState = 0;
                    transfer.setPower(0);
                    intake.setPower(0.5);
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

}
