package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Voyager Blue", group = "Autonomous")
public class VoyagerBlue extends LinearOpMode {

    private final Pose park = new Pose(36, 10, Math.toRadians(270));
    AutoUtil au;


    @Override
    public void runOpMode() {

        au = new AutoUtil(this, AutoUtil.Side.BLUEFAR);

        waitForStart();

        au.start();

        // Park
        au.goToPoint(park, 4);
        au.updateSharedHeading();

    }

}


