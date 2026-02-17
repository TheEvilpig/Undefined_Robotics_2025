package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Hammer Red", group = "Autonomous")
public class HammerRed extends LinearOpMode {

    private final Pose park = new Pose(108, 10, Math.toRadians(270));
    AutoUtil au;


    @Override
    public void runOpMode() {

        au = new AutoUtil(this, AutoUtil.Side.REDFAR);

        waitForStart();

        au.start();

        au.scorePreloads(3.5);

        au.intakeFar(1.75, 1.75);

        // take an extra path to not hit balls
        au.setShooterVelocity(au.config.closeVelocity());

        au.followTwoPointPath(au.config.farIntakeEnd(), au.config.farIntakeStart(), .7);
        au.followTwoPointPath(au.config.farIntakeStart(), au.config.farShooting(), 2.5);

        au.shootSequence();

        // Park
        au.goToPoint(park, 2.5);
        au.updateSharedHeading();

    }

}


