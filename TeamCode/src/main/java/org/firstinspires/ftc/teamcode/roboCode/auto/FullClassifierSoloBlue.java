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

    private final Pose start = new Pose(60, 9, Math.toRadians(270));

    private final Pose farShooting = new Pose(60, 16, Math.toRadians(293));
    private final Pose closeShooting = new Pose(56, 80, Math.toRadians(131));

    private final Pose farIntakeStart = new Pose(48, 36, Math.toRadians(180));
    private final Pose farIntakeEnd = new Pose(22, 36, Math.toRadians(180));

    private final Pose midIntakeStart = new Pose(48, 60, Math.toRadians(180));
    private final Pose midIntakeEnd = new Pose(22, 60, Math.toRadians(180));

    private final Pose closeIntakeStart = new Pose(48, 84, Math.toRadians(180));
    private final Pose closeIntakeEnd = new Pose(22, 84, Math.toRadians(180));

    private final Pose park = new Pose(22, 84, Math.toRadians(180));

    private final double FAR_SHOOTING_VELOCITY = 390;
    private final double CLOSE_SHOOTING_VELOCITY = 330;

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
        shooter.setTargetVelocity(400);
        followTwoPointPath(start, farShooting, 1);
        hold.setPosition(HConst.HOLD_INACTIVE);







    }

    private void followTwoPointPath(Pose p1, Pose p2, double t){
        pathToTarget = follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
                .build();

        double ct = timer.getElapsedTimeSeconds();
        while(timer.getElapsedTimeSeconds() < ct + t){
            follower.update();
            shooter.update();
        }

    }








}


