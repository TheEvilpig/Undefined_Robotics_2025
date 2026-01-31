package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.DcMotorSystem;
import org.firstinspires.ftc.teamcode.util.HConst;


@Autonomous(name="Full Classifier Solo Blue", group = "Autonomous")
public class FullClassifierSoloBlue extends LinearOpMode {

    private final Pose start = new Pose(60, 8, Math.toRadians(270));

    private final Pose farShooting = new Pose(60, 13, Math.toRadians(295.5));
    private final Pose farShooting2 = new Pose(60, 13, Math.toRadians(291));
    private final Pose closeShooting = new Pose(65, 73, Math.toRadians(311.5));
    private final Pose closeShooting2 = new Pose(65, 73, Math.toRadians(307.5));

    private final Pose farIntakeStart = new Pose(48, 49, Math.toRadians(180));
    private final Pose farIntakeEnd = new Pose(23, 49, Math.toRadians(180));

    private final Pose midIntakeStart = new Pose(64, 68, Math.toRadians(180));
    private final Pose midIntakeEnd = new Pose(18, 72, Math.toRadians(180));

    private final Pose closeIntakeStart = new Pose(48, 94, Math.toRadians(180));
    private final Pose closeIntakeEnd = new Pose(28, 94, Math.toRadians(180));

    private final Pose park = new Pose(28, 72, Math.toRadians(270));

    private final double FAR_SHOOTING_VELOCITY = 327;
    private final double CLOSE_SHOOTING_VELOCITY = 250;

    // Shooting sequence timing constants
    private final double SHOOT_SPINUP_TIME = 0.75;  // Time for shooter to reach velocity
    private final double SHOOT_PULSE_DURATION = 0.12;  // Time hold is open
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
    private PathChain pathToTarget1;

    @Override
    public void runOpMode() {
        //shooting system
        intake = hardwareMap.get(DcMotorEx.class, HConst.INTAKE);

        //outtake 1 is in same orientation as previous outtake motor
        outtake = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);
        transfer = hardwareMap.get(DcMotorEx.class, HConst.TRANSFER);

        hold = hardwareMap.get(Servo.class, HConst.HOLD);

        intake.setDirection(HConst.INTAKE_DIR);
        outtake.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer.setDirection(HConst.TRANSFER_DIR);

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
        followTwoPointPath(start, farShooting, 3);
        shootSequence(FAR_SHOOTING_VELOCITY, 3);

        followTwoPointPath(farShooting, farIntakeStart, 2.9);
        intake.setPower(1);
        transfer.setPower(1);
        followTwoPointPath(farIntakeStart, farIntakeEnd, 2.5);
        intake.setPower(0.5);
        transfer.setPower(0);

        // Return to far shooting and shoot
        shooter.setTargetVelocity(CLOSE_SHOOTING_VELOCITY);
        followTwoPointPath(farIntakeEnd, farIntakeStart, 1);
        followTwoPointPath(farIntakeStart, closeShooting, 5);
        shootSequence(CLOSE_SHOOTING_VELOCITY, 3);

        // Intake second line
        followTwoPointPath(farShooting, midIntakeStart, 3.5);
        intake.setPower(1);
        transfer.setPower(1);
        followTwoPointPath(midIntakeStart, midIntakeEnd, 2);
        intake.setPower(0.5);
        transfer.setPower(0);

        // Move to close shooting position and shoot
        shooter.setTargetVelocity(CLOSE_SHOOTING_VELOCITY);
        //followTwoPointPath(midIntakeEnd, midIntakeStart, 1);
        followTwoPointPath(midIntakeStart, closeShooting, 5);
        shootSequence(CLOSE_SHOOTING_VELOCITY, 3);

        // Intake third line
        followTwoPointPath(closeShooting, closeIntakeStart, 2.5);
        intake.setPower(1);
        transfer.setPower(1);
        followTwoPointPath(closeIntakeStart, closeIntakeEnd, 1.5);
        intake.setPower(0.5);
        transfer.setPower(0);

        // Return to close shooting and shoot
        shooter.setTargetVelocity(CLOSE_SHOOTING_VELOCITY);
        followTwoPointPath(closeIntakeEnd, closeShooting2, 4.5);
        shootSequence(CLOSE_SHOOTING_VELOCITY, 3);

        // Park
        followTwoPointPath(closeShooting, park, 1);

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


