package org.firstinspires.ftc.teamcode.stemos.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.stemos.subsystem.Mecanum;

public class DeadReckoning implements Action {
    private final double maxTime;
    private final double distance;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean initialized = false;
    private final Mecanum drivetrain;

    public DeadReckoning(Mecanum drivetrain, double distance)
    {
        this.drivetrain = drivetrain;
        this.distance = distance;
        maxTime = distance / Mecanum.MAX_VELOCITY;
    }


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        telemetryPacket.put("Target distance", distance);
        telemetryPacket.put("Actual distance", drivetrain.getLinearDistanceOfOneMotor());

        if(!initialized)
        {
            timer.reset();
            initialized = true;
        }
        if(timer.seconds() <= maxTime)
        {
            drivetrain.drive(1, 0, 0);
        } else {
            drivetrain.drive(0, 0, 0);
            return false;
        }
        return true;
    }
}
