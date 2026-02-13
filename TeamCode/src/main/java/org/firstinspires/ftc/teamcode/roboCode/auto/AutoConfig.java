package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.pedropathing.geometry.Pose;

public interface AutoConfig {
    Pose start();
    Pose farShooting();
    Pose closeShooting();
    Pose farIntakeStart();
    Pose farIntakeEnd();
    Pose midIntakeStart();
    Pose midIntakeEnd();
    Pose closeIntakeStart();
    Pose closeIntakeEnd();

    double farVelocity();
    double closeVelocity();
}

