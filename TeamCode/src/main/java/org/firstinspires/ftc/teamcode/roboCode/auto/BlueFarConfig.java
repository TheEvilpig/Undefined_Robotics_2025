package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.geometry.Pose;

public class BlueFarConfig implements AutoConfig {

    @Override
    public Pose start() {
        return new Pose(60, 8, Math.toRadians(270));
    }

    @Override
    public Pose farShooting() {
        return new Pose(60, 13, Math.toRadians(295.5));
    }

    @Override
    public Pose closeShooting() {
        return new Pose(65, 73, Math.toRadians(311.5));
    }

    @Override
    public Pose farIntakeStart() {
        return new Pose(48, 49, Math.toRadians(180));
    }

    @Override
    public Pose farIntakeEnd() {
        return new Pose(23, 49, Math.toRadians(180));
    }

    @Override
    public Pose midIntakeStart() {
        return new Pose(64, 68, Math.toRadians(180));
    }

    @Override
    public Pose midIntakeEnd() {
        return new Pose(18, 72, Math.toRadians(180));
    }

    @Override
    public Pose closeIntakeStart() {
        return new Pose(48, 94, Math.toRadians(180));
    }

    @Override
    public Pose closeIntakeEnd() {
        return new Pose(28, 94, Math.toRadians(180));
    }

    /*@Override
    public Pose park() {
        return new Pose(28, 72, Math.toRadians(270));
    }*/

    @Override
    public double farVelocity() {
        return 339;
    }

    @Override
    public double closeVelocity() {
        return 305;
    }
}
