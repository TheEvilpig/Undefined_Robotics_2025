package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.geometry.Pose;

public class BlueFarConfig implements AutoConfig {

    @Override
    public Pose start() {
        return new Pose(60, 8, Math.toRadians(270));
    }

    @Override
    public Pose farShooting() {
        return new Pose(60,
                14, Math.toRadians(296));
    }

    @Override
    public Pose closeShooting() {
        return new Pose(61, 84, Math.toRadians(318.5));
    }

    @Override
    public Pose farIntakeStart() {
        return new Pose(52, 48, Math.toRadians(180));
    }

    @Override
    public Pose farIntakeEnd() {
        return new Pose(29, 48, Math.toRadians(180));
    }

    @Override
    public Pose midIntakeStart() {
        return new Pose(52, 71, Math.toRadians(180));
    }

    @Override
    public Pose midIntakeEnd() {
        return new Pose(23, 71, Math.toRadians(180));
    }

    @Override
    public Pose closeIntakeStart() {
        return new Pose(52, 94, Math.toRadians(180));
    }

    @Override
    public Pose closeIntakeEnd() {
        return new Pose(26, 94, Math.toRadians(180));
    }

    /*@Override
    public Pose park() {
        return new Pose(28, 72, Math.toRadians(270));
    }*/
    public Pose humanPlayerStart(){return new Pose(26,28, 240);}

    public Pose humanPlayerEnd(){return new Pose(26, 8, 240);}

    public Pose secretTunnel(){return new Pose();}

    @Override
    public double farVelocity() {
        return 348;
    }

    @Override
    public double closeVelocity() {
        return 297.25;
    }
}
