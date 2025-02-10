package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PIDCommand implements Action {

    private final double distance;
    private final Mecanum drivetrain;
    private boolean initialized = false;

    public PIDCommand(Mecanum drivetrain, double distance)
    {
        this.drivetrain = drivetrain;
        this.distance = distance;

        drivetrain.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public PIDCommand(Mecanum drivetrain, double distance, PIDFCoefficients pidf)
    {
        this.drivetrain = drivetrain;
        this.distance = distance;

        drivetrain.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.setPIDF(pidf.p, pidf.i, pidf.d, pidf.f);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(!initialized)
        {
            initialized = true;
            drivetrain.setLinearTarget(distance);
            drivetrain.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        drivetrain.drive(1, 0, 0);
        return drivetrain.atTarget();
    }
}
