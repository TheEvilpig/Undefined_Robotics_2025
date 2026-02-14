package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Full Classifier Solo Blue", group = "Autonomous")
public class FullClassifierSoloBlue extends LinearOpMode {
    AutoUtil au;
    Pose park = new Pose(28, 72, Math.toRadians(270));

    @Override
    public void runOpMode() {


        au = new AutoUtil(this, AutoUtil.Side.BLUEFAR);

        waitForStart();

        au.start();

        au.scorePreloads(3.0);

        au.intakeFar(1.25, 1.25);

        // take an extra path to not hit balls
        au.setShooterVelocity(au.config.closeVelocity());

        au.followTwoPointPath(au.config.farIntakeEnd(), au.config.farIntakeStart(), 1);
        au.followTwoPointPath(au.config.farIntakeStart(), au.config.closeShooting(), 2.5);

        au.shootSequence();

        au.intakeMid(1.25, 1.25);

        au.scoreClose(2.5);

        au.intakeClose(1.25, 1.25);

        au.scoreClose(2.5);

        au.setShooterVelocity(0);

        // Park
        au.goToPoint(park, 1);
        au.updateSharedHeading();


    }
}