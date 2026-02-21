package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.DcMotorSystem;
import org.firstinspires.ftc.teamcode.util.HConst;
import org.firstinspires.ftc.teamcode.util.MatchState;

public class AutoUtil {
    public final double SHOOT_TIME = 1;
    public final double DEFAULT_VELOCITY = 250;
    public final double FAR_SHOOTING_POS = 0.5;
    public final double CLOSE_SHOOTING_POS = 0.5;

    AutoConfig config;
    DcMotorEx outtake;
    DcMotorEx outtake2;
    DcMotorEx transfer1, transfer2;
    Follower follower;

    private IMU imu;

    private Servo hold;

    private Servo intake1;
    private Servo intake2;

    private Servo hood;


    private Timer timer;

    private DcMotorSystem shooter;

    LinearOpMode opMode;


    public AutoUtil(LinearOpMode opMode, Side s) {
        this.opMode = opMode;

        if(s == Side.REDFAR){
            config = new RedFarConfig();
        }

        if(s == Side.BLUEFAR){
            config = new BlueFarConfig();
        }

        HardwareMap hw = opMode.hardwareMap;

        // -----------------------
        // Hardware Mapping
        // -----------------------

        outtake = hw.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hw.get(DcMotorEx.class, HConst.OUTTAKE2);
        transfer1 = hw.get(DcMotorEx.class, HConst.TRANSFER1);
        transfer2 = hw.get(DcMotorEx.class,HConst.TRANSFER2);

        hold = hw.get(Servo.class, HConst.HOLD);
        intake1 = hw.get(Servo.class,HConst.INTAKE_LEFT_SERVO);
        intake2 = hw.get(Servo.class,HConst.INTAKE_RIGHT_SERVO);
        hood = hw.get(Servo.class,HConst.HOOD);

        outtake.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer1.setDirection(HConst.TRANSFER1_DIR);
        transfer2.setDirection(HConst.TRANSFER2_DIR);
        // -----------------------
        // Shooter System
        // -----------------------

        shooter = new DcMotorSystem(
                outtake2,
                28,
                0.01,
                opMode.telemetry
        );

        shooter.addFollower(outtake);

        shooter.setPID(
                0.00645,
                0.00015,
                0.00175,
                0.1358
        );

        shooter.setTargetVelocity(0);

        // -----------------------
        // Follower
        // -----------------------

        follower = Constants.createFollower(hw);
        follower.setStartingPose(config.start());

        // -----------------------
        // Timer
        // -----------------------

        timer = new Timer();
        timer.resetTimer();

        // -----------------------
        // IMU
        // -----------------------

        imu = hw.get(IMU.class, "IMU");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();


    }

    public void start(){
        shooter.enable();
        setIdlePowers();
        setShooterVelocity(DEFAULT_VELOCITY);
    }

    public void setShooterVelocity(double x){
        shooter.setTargetVelocity(x);
    }

    public void followTwoPointPath(Pose p1, Pose p2, double t){
        PathChain pathToTarget = follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
                .build();

        follower.followPath(pathToTarget);

        double ct = timer.getElapsedTimeSeconds();
        while (opMode.opModeIsActive() && (follower.isBusy() || timer.getElapsedTimeSeconds() < ct + t)) {
            follower.update();
            shooter.update();
        }

    }

    public void goToPoint(Pose p2, double t){
        Pose p1 = follower.getPose();
        PathChain pathToTarget = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), p2))
                .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
                .build();

        follower.followPath(pathToTarget);

        double ct = timer.getElapsedTimeSeconds();
        while (opMode.opModeIsActive() && follower.isBusy() && timer.getElapsedTimeSeconds() < ct + t) {
            follower.update();
            shooter.update();
        }
    }

    public void shootSequence() {
        hold.setPosition(HConst.HOLD_INACTIVE);

        double startTime = timer.getElapsedTimeSeconds();

        transfer1.setPower(0.8);
        transfer2.setPower(0.8);
        while (opMode.opModeIsActive() &&
                (timer.getElapsedTimeSeconds() - startTime) < SHOOT_TIME) {
            shooter.update();
            follower.update();
        }

        hold.setPosition(HConst.HOLD_ACTIVE);
        transfer1.setPower(0);
        transfer2.setPower(0);
    }

    public void setIntakePowers() {
        intake1.setPosition(HConst.INTAKE1DOWN);
        intake2.setPosition(HConst.INTAKE2DOWN);
        transfer1.setPower(0.85);
        transfer2.setPower(0.85);
    }

    public void setIdlePowers() {
        intake1.setPosition(HConst.INTAKE1UP);
        intake2.setPosition(HConst.INTAKE2UP);
        transfer1.setPower(0.1);
        transfer2.setPower(0.1);
    }

    public void scoreFar(double t){
        hood.setPosition(FAR_SHOOTING_POS);
        setShooterVelocity(config.farVelocity());
        followTwoPointPath(follower.getPose(), config.farShooting(), t);
        shootSequence();
        setShooterVelocity(DEFAULT_VELOCITY);
    }

    public void scoreClose(double t){
        hood.setPosition(CLOSE_SHOOTING_POS);
        setShooterVelocity(config.closeVelocity());
        followTwoPointPath(follower.getPose(), config.closeShooting(), t);
        shootSequence();
        setShooterVelocity(DEFAULT_VELOCITY);
    }

    public void intakeFar(double t, double t2){
        followTwoPointPath(follower.getPose(), config.farIntakeStart(), t);
        setIntakePowers();

        followTwoPointPath(config.farIntakeStart(), config.farIntakeEnd(), t2);
        setIdlePowers();
    }

    public void intakeMid(double t, double t2){
        followTwoPointPath(follower.getPose(), config.midIntakeStart(), t);
        setIntakePowers();

        followTwoPointPath(config.midIntakeStart(), config.midIntakeEnd(), t);
        setIdlePowers();
    }

    public void intakeClose(double t, double t2){
        followTwoPointPath(follower.getPose(), config.closeIntakeStart(), 2.5);
        setIntakePowers();

        followTwoPointPath(config.closeIntakeStart(), config.closeIntakeEnd(), 1.5);
        setIdlePowers();
    }

    public void updateSharedHeading(){
        double headingDeg =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        MatchState.finalAutoHeadingDeg = headingDeg;
    }

    public enum Side {
        BLUEFAR,
        REDFAR,
        BLUECLOSE,
        REDCLOSE
    }
    
}
