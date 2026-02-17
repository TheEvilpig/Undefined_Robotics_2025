package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Jasper Blue", group = "Autonomous")
public class JasperBlue extends LinearOpMode {
    AutoUtil au;
    Pose park = new Pose(40, 72, Math.toRadians(270));

    @Override
    public void runOpMode() {


        au = new AutoUtil(this, AutoUtil.Side.BLUEFAR);

        waitForStart();

        au.start();

        au.scorePreloads(3.0);

        au.goToPoint(au.config.humanPlayerStart(), 2);
        au.setIntakePowers();
        au.goToPoint(au.config.humanPlayerEnd(), 2);
        au.setIdlePowers();
        au.setShooterVelocity(au.config.farVelocity());
        au.goToPoint(au.config.farShooting(), 5);
        au.shootSequence();

        // Park
        au.goToPoint(park, .8);
        au.updateSharedHeading();


    }
}