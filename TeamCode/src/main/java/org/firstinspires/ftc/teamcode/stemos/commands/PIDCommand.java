package org.firstinspires.ftc.teamcode.stemos.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.stemos.subsystem.ArmSubsystem;
import org.firstinspires.ftc.teamcode.stemos.subsystem.Mecanum;

public class PIDCommand implements Action {

    private final double distance;
    private final ArmSubsystem arm;
    private boolean initialized = false;

    public PIDCommand(ArmSubsystem arm, double distance)
    {
        this.arm = arm;
        this.distance = distance;
    }

    public PIDCommand(ArmSubsystem arm, double distance, PIDFCoefficients pidf)
    {
        this.arm = arm;
        this.distance = distance;

        arm.setPIDFForPosition(pidf.p, pidf.i, pidf.d);
        arm.setPIDFForVelocity(pidf.p, pidf.i, pidf.d, pidf.i);
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
        return true;
    }
}
