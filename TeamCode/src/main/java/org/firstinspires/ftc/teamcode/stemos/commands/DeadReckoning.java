package org.firstinspires.ftc.teamcode.stemos.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.stemos.subsystem.ArmSubsystem;

public class DeadReckoning implements Action {
    private final double maxTime;
    private final double distance;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean initialized = false;
    private final ArmSubsystem arm;

    public DeadReckoning(ArmSubsystem arm, double distance)
    {
        this.arm = arm;
        this.distance = distance;
        maxTime = distance / ArmSubsystem.MAX_VELOCITY;
    }


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        telemetryPacket.put("Target distance", distance);
        telemetryPacket.put("Actual distance", arm.getAngleOfArm());

        if(!initialized)
        {
            timer.reset();
            initialized = true;
        }
        if(timer.seconds() <= maxTime)
        {
            arm.setSpeed(1.0);
        } else {
            arm.setSpeed(0);
            return false;
        }
        return true;
    }
}
