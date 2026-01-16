package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.DcMotorSystem;
import org.firstinspires.ftc.teamcode.util.HConst;

@Autonomous(name="Main Autonomous Red",group = "Autonomous")
public class mainAutoRed extends LinearOpMode {
    private DcMotorEx frontLeftDrive;
    private DcMotorEx backLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx backRightDrive;
    private DcMotorEx intake, outtake1, outtake2;
    private DcMotorSystem shooter;
    private DcMotorEx transfer;
    private Follower follower;
    public static String seq;
    private IMU imu;
    private Limelight3A limeLight;
    private LLResult latestResult = null;
    private GoBildaPinpointDriver odoComputer;

    private Timer pathTimer;
    private int pathState;
    private int shootingState = 0;
    private int ballsShot = 0;
    private double distance = -1;

    // Power variables for intake, outtake, and transfer
    private double intakePower = 1;
    private double outtakePower = 0.98;
    private double transferPower = 1;
    private double targetSpeed = 0.0;

    // Define poses
    private final Pose startPose = new Pose(84, 12, Math.toRadians(90));
    private final Pose targetPose = new Pose(84, 5, Math.toRadians(67));

    private final Pose targetPose1 = new Pose(84, -20, Math.toRadians(67));


    // Define path
    private PathChain pathToTarget;
    private PathChain pathToTarget1;

    @Override
    public void runOpMode(){
        intake = hardwareMap.get(DcMotorEx.class,"Intake");
        outtake1 = hardwareMap.get(DcMotorEx.class,"Outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");
        transfer = hardwareMap.get(DcMotorEx.class,"Transfer");

        intake.setDirection(HConst.INTAKE_DIR);
        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer.setDirection(HConst.TRANSFER_DIR);

        shooter = new DcMotorSystem(
                outtake2,
                28,     // ticks per rev
                0.1, // velocity update interval
                telemetry
        );
        shooter.addFollower(outtake1);
        shooter.setPID(
                0.015,  // kP
                0.0008,  // kI
                0.002,  // kF
                0.2     // kStatic
        );
        shooter.setTargetVelocity(0);
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
            // Update limelight
            //scanAprilTags();
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
                    // need to implement auto turning
                    shooter.setTargetVelocity(shooter.computeTargetVelocityFromDistance(distance));

                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() >= 3.5){
                    shooter.setTargetVelocity(0);
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
                    shooter.setTargetVelocity(0.0);
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

            distance = getDistance(result);
            latestResult = result;

        } else {
            latestResult = null;

            telemetry.addLine("No targets detected from Limelight.");
        }
    }*/

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
    }*/

}
