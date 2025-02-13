package org.firstinspires.ftc.teamcode.stemos.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.stemos.Constants;

public class ArmSubsystem {
    private static final double CPR = 1680;
    private static final double REDUCTION = 1;
    public static final double MAX_VELOCITY = 599.695826;

    private final DcMotorEx pivot;
    private double factorConversionEncoder = 1;


    public ArmSubsystem(OpMode opMode)
    {
        pivot = opMode.hardwareMap.get(DcMotorEx.class, Constants.ArmNames.ARM);
        setConversionFactorEncoders(360.0 / (REDUCTION * CPR));

        setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPIDFForPosition(double p, double i, double d)
    {
        PIDFCoefficients pidfNew = new PIDFCoefficients(p, i, d, 0);
        DcMotorEx[] motors = {pivot};
        DcMotorControllerEx[] motorControllerEx = new DcMotorControllerEx[4];
        for(int count = 0; count < motors.length; count++)
        {
            motorControllerEx[count] = (DcMotorControllerEx) motors[count].getController();
            motorControllerEx[count].setPIDFCoefficients(motors[count].getPortNumber(), DcMotor.RunMode.RUN_TO_POSITION, pidfNew);
        }
    }

    public void setPIDFForVelocity(double p, double i, double d, double f)
    {
        PIDFCoefficients pidfNew = new PIDFCoefficients(p, i, d, f);
        DcMotorEx[] motors = {pivot};
        DcMotorControllerEx[] motorControllerEx = new DcMotorControllerEx[4];
        for(int count = 0; count < motors.length; count++)
        {
            motorControllerEx[count] = (DcMotorControllerEx) motors[count].getController();
            motorControllerEx[count].setPIDFCoefficients(motors[count].getPortNumber(), DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        }
    }

    public void resetArmEncoders()
    {
        setArmMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getAngleOfArm()
    {
        return pivot.getCurrentPosition() * factorConversionEncoder;
    }

    public void setAngleOfArm(double target)
    {
        DcMotorEx[] motors = {pivot};
        for(DcMotorEx motor : motors)
        {
            motor.setTargetPosition((int)(target / factorConversionEncoder));
        }
        setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        setVelocity();
    }

    public void setArmMode(DcMotor.RunMode modo)
    {
        DcMotorEx[] motors = {pivot};
        for(DcMotorEx motor : motors)
        {
            motor.setMode(modo);
        }
    }


    private void setVelocity()
    {
        DcMotorEx[] motors = {pivot};
        for(DcMotorEx motor : motors)
        {
            motor.setPower(0.7);
        }
    }

    public void setSpeed(double power)
    {
        DcMotorEx[] motors = {pivot};
        for(DcMotorEx motor : motors)
        {
            motor.setPower(power);
        }
    }

    public boolean atTarget()
    {
        return !pivot.isBusy();
    }

    public void setConversionFactorEncoders(double factor)
    {
        factorConversionEncoder = factor;
    }

    public Action setArmAngle(double angle) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    setAngleOfArm(angle);
                    initialized = true;
                }

                packet.put("Angle of Arm", getAngleOfArm());
                packet.put("Command finished", !(getAngleOfArm() < angle));

                return !atTarget();
            }
        };
    }
}
