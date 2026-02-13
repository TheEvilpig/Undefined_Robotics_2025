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

        au.scorePreloads(3.6);

        au.intakeFar(1.3, 1.8);

        // take an extra path to not hit balls
        au.setShooterVelocity(au.config.closeVelocity());

        au.followTwoPointPath(au.config.farIntakeEnd(), au.config.farIntakeStart(), 0.2);
        au.followTwoPointPath(au.config.farIntakeStart(), au.config.closeShooting(), 2);

        au.shootSequence();

        au.intakeMid(1.1, 1.5);
        au.goToPoint(au.config.midIntakeStart(), 0.3);

        au.scoreClose(2.3);

        au.intakeClose(1.1, 1.1);

        au.scoreClose(2);

        au.setShooterVelocity(0);

        // Park
        au.goToPoint(park, 0.8);
        au.updateSharedHeading();

    }
}