package org.firstinspires.ftc.teamcode.stemos.commands;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.stemos.subsystem.Mecanum;

public class PIDCommand implements Action {

    private final double distance;
    private final Mecanum drivetrain;
    private boolean initialized = false;

    public PIDCommand(Mecanum drivetrain, double distance)
    {
        this.drivetrain = drivetrain;
        this.distance = distance;

        drivetrain.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivetrain.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public PIDCommand(Mecanum drivetrain, double distance, PIDFCoefficients pidf)
    {
        this.drivetrain = drivetrain;
        this.distance = distance;

        drivetrain.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivetrain.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.setPIDFForPosition(pidf.p, pidf.i, pidf.d);
        drivetrain.setPIDFForVelocity(pidf.p, pidf.i, pidf.d, pidf.i);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        telemetryPacket.put("Target", distance);
        telemetryPacket.put("Actual Pose", drivetrain.getLinearDistanceOfOneMotor());

        if(!initialized)
        {
            initialized = true;
            drivetrain.setLinearTarget(distance);
            drivetrain.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        drivetrain.drive(0.7, 0, 0);
        return true;
    }
}
