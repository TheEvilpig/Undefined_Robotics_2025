/*
        package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.DcMotorSystem;
import org.firstinspires.ftc.teamcode.util.HConst;
import org.firstinspires.ftc.teamcode.util.MatchState;


@Autonomous(name="Far Red", group = "Autonomous")
public class FarRed extends LinearOpMode {

    private final Pose start = new Pose(84, 8, Math.toRadians(270));

    private final Pose farShooting = new Pose(84, 25.5, Math.toRadians(243));
    private final Pose farShooting2 = new Pose(84, 24.5, Math.toRadians(243));

    private final Pose farIntakeStart = new Pose(93, 26, Math.toRadians(0));
    private final Pose farIntakeEnd = new Pose(148, 26, Math.toRadians(0));

    private final Pose park = new Pose(108, 10, Math.toRadians(270));


    private final double FAR_SHOOTING_VELOCITY = 325;
    private final double FAR_SHOOTING_VELOCITY2 = 325;
    private final double CLOSE_SHOOTING_VELOCITY = 259;

    // Shooting sequence timing constants
    private final double SHOOT_SPINUP_TIME = 0.75;  // Time for shooter to reach velocity
    private final double SHOOT_PULSE_DURATION = 0.125;  // Time hold is open
    private final double SHOOT_PULSE_SPACING = 0.75;  // Time between pulses


    DcMotorEx intake;
    DcMotorEx outtake;
    DcMotorEx outtake2;
    DcMotorEx transfer;
    Follower follower;

    private Servo hold;

    private Timer timer;

    private DcMotorSystem shooter;

    // Define path
    private PathChain pathToTarget;

    private IMU imu;
    private PathChain pathToTarget1;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "IMU");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();
        //shooting system
        intake = hardwareMap.get(DcMotorEx.class, HConst.INTAKE);

        //outtake 1 is in same orientation as previous outtake motor
        outtake = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);
        transfer = hardwareMap.get(DcMotorEx.class, HConst.TRANSFER2);

        hold = hardwareMap.get(Servo.class, HConst.HOLD);

        intake.setDirection(HConst.INTAKE_DIR);
        outtake.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer.setDirection(HConst.TRANSFER2_DIR);

        shooter = new DcMotorSystem(
                outtake2,
                28,     // ticks per rev
                0.01, // velocity update interval
                telemetry
        );

        // Initialize shooter system
        shooter.addFollower(outtake);
        shooter.setPID(
                0.00345,  // kP
                0.00015,  // kI
                0.00175,  // kF
                0.1358     // kStatic
        );
        shooter.setTargetVelocity(0);

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);

        // Initialize timer
        timer = new Timer();
        timer.resetTimer();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Position", "X: %.1f, Y: %.1f", start.getX(), start.getY());
        telemetry.update();

        hold.setPosition(HConst.HOLD_ACTIVE);

        waitForStart();

        shooter.enable();

        shooter.setTargetVelocity(FAR_SHOOTING_VELOCITY);
        intake.setPower(0.5);
        followTwoPointPath(start, farShooting, 5);
        shootSequence(FAR_SHOOTING_VELOCITY, 3);
        double headingDeg =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        MatchState.finalAutoHeadingDeg = headingDeg;

    }


    private void shootSequence(double velocity, int numPieces) {

        // Start shooter and hold closed
        shooter.setTargetVelocity(velocity);
        hold.setPosition(HConst.HOLD_ACTIVE);
        intake.setPower(1);
        transfer.setPower(0.75);
        double startTime = timer.getElapsedTimeSeconds();
        double currentTime;

        // Spin-up phase - wait for shooter to reach velocity
        while (opModeIsActive() &&
                (timer.getElapsedTimeSeconds() - startTime) < SHOOT_SPINUP_TIME) {
            shooter.update();
            follower.update();
        }

        // Shooting phase - pulse the hold servo to release pieces
        for (int i = 0; i < numPieces; i++) {
            if (!opModeIsActive()) break;

            // Open hold to release piece
            hold.setPosition(HConst.HOLD_INACTIVE);
            startTime = timer.getElapsedTimeSeconds();
            while (opModeIsActive() &&
                    (timer.getElapsedTimeSeconds() - startTime) < SHOOT_PULSE_DURATION) {
                shooter.update();
                follower.update();
            }

            // Close hold
            hold.setPosition(HConst.HOLD_ACTIVE);

            // Wait before next piece (if not last piece)
            if (i < numPieces - 1) {
                startTime = timer.getElapsedTimeSeconds();
                while (opModeIsActive() &&
                        (timer.getElapsedTimeSeconds() - startTime) < SHOOT_PULSE_SPACING) {
                    shooter.update();
                    follower.update();
                }
            }
        }

        // Stop shooter
        shooter.setTargetVelocity(0);
        intake.setPower(0.5);
        transfer.setPower(0);
    }

    private void followTwoPointPath(Pose p1, Pose p2, double t){
        pathToTarget = follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
                .build();

        follower.followPath(pathToTarget);

        double ct = timer.getElapsedTimeSeconds();
        while (opModeIsActive() && follower.isBusy() && timer.getElapsedTimeSeconds() < ct + t) {
            follower.update();
            shooter.update();
        }

    }








}

 */


