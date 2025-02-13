package org.firstinspires.ftc.teamcode.stemos.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.stemos.subsystem.ArmSubsystem;

public class PIDCommand implements Action {

    private final double distance;
    private final ArmSubsystem arm;
    private boolean initialized = false;
    ElapsedTime timer;

    public PIDCommand(ArmSubsystem arm, double distance)
    {
        this.arm = arm;
        this.distance = distance;
        timer = new ElapsedTime();
        timer.reset();
        arm.resetArmEncoders();
    }

    public PIDCommand(ArmSubsystem arm, double distance, PIDFCoefficients pidf)
    {
        this.arm = arm;
        this.distance = distance;
        timer = new ElapsedTime();
        timer.reset();
        arm.setPIDFForPosition(pidf.p, pidf.i, pidf.d);
        arm.setPIDFForVelocity(pidf.p, pidf.i, pidf.d, pidf.i);
        arm.resetArmEncoders();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        telemetryPacket.put("Target", distance);
        telemetryPacket.put("Actual Pose", arm.getAngleOfArm());

        if(!initialized)
        {
            initialized = true;
            arm.setAngleOfArm(distance);
        }
        return timer.seconds() < 4.0;
    }
}
