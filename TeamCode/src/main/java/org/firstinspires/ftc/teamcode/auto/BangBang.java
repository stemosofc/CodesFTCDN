package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class BangBang implements Action {

    private final double targetDistance;
    private final Mecanum drivetrain;

    public BangBang(Mecanum drivetrain, double distance)
    {
            this.drivetrain = drivetrain;
            targetDistance = distance;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double error = targetDistance - drivetrain.getLinearDistance();
        if(error >= 0)
        {
            drivetrain.drive(1, 0, 0);
        } else if(error <= 0)
        {
            drivetrain.drive(-1, 0, 0);
        }
        return error == 0;
    }
}
