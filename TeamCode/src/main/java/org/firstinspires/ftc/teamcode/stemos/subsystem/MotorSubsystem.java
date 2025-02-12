package org.firstinspires.ftc.teamcode.stemos.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.stemos.Constants;

public class MotorSubsystem {
    public static final double MAX_VELOCITY = 12; // in/s
    private static final double CPR = 1680;
    private static final double WHEEL_DIAMETER = 4;

    private DcMotorEx motor;
    private double factorConversionEncoder = 1;

    public MotorSubsystem(OpMode opMode)
    {
        motor = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.MOTOR_ESQUERDA_FRENTE);
        setConversionFactorEncoders(WHEEL_DIAMETER * Math.PI / CPR);
    }

    public void setPIDFForPosition(double p, double i, double d)
    {
        PIDFCoefficients pidfNew = new PIDFCoefficients(p, i, d, 0);
        DcMotorEx[] motors = {motor};
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
        DcMotorEx[] motors = {motor};
        DcMotorControllerEx[] motorControllerEx = new DcMotorControllerEx[4];
        for(int count = 0; count < motors.length; count++)
        {
            motorControllerEx[count] = (DcMotorControllerEx) motors[count].getController();
            motorControllerEx[count].setPIDFCoefficients(motors[count].getPortNumber(), DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        }
    }

    public double getLinearDistanceOfOneMotor()
    {
        return motor.getCurrentPosition() * factorConversionEncoder;
    }

    public void setLinearTarget(double target)
    {
        DcMotorEx[] motors = {motor};
        for(DcMotorEx motor : motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + (int)(target / factorConversionEncoder));
        }
    }

    public boolean atTarget()
    {
        return motor.isBusy();
    }

    public void setConversionFactorEncoders(double factor)
    {
        factorConversionEncoder = factor;
    }

}
