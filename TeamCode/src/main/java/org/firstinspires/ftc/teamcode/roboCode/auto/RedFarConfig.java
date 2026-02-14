package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.geometry.Pose;

public class RedFarConfig implements AutoConfig {

    @Override
    public Pose start() {
        return new Pose(84, 8, Math.toRadians(270));
    }

    @Override
    public Pose farShooting() {
        return new Pose(84, 25.5, Math.toRadians(243));
    }

    @Override
    public Pose closeShooting() {
        return new Pose(83, 84, Math.toRadians(227.5));
    }

    @Override
    public Pose farIntakeStart() {
        return new Pose(93, 26, Math.toRadians(0));
    }

    @Override
    public Pose farIntakeEnd() {
        return new Pose(148, 26, Math.toRadians(0));
    }

    @Override
    public Pose midIntakeStart() {
        return new Pose(99, 50.5, Math.toRadians(0));
    }

    @Override
    public Pose midIntakeEnd() {
        return new Pose(148, 46, Math.toRadians(0));
    }

    @Override
    public Pose closeIntakeStart() {
        return new Pose(93, 74.5, Math.toRadians(0));
    }

    @Override
    public Pose closeIntakeEnd() {
        return new Pose(143, 72, Math.toRadians(0));
    }

    @Override
    public double farVelocity() {
        return 339;
    }

    @Override
    public double closeVelocity() {
        return 305;
    }
}
