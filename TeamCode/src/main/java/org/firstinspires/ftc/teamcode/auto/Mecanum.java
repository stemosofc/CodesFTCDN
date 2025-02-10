package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Mecanum {
    public static final double MAX_VELOCITY = 12; // in/s
    private static final double CPR = 1680;
    private static final double WHEEL_DIAMETER = 4;

    private final DcMotorEx motorEsquerdaFrente;
    private final DcMotorEx motorDireitaFrente;
    private final DcMotorEx motorEsquerdaTras;
    private final DcMotorEx motorDireitaTras;

    private double factorConversionEncoder = 1;

    public Mecanum(OpMode opMode) {
        motorEsquerdaFrente = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorEsquerdaFrente);
        motorDireitaFrente = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorDireitaFrente);
        motorEsquerdaTras = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorEsquerdaTras);
        motorDireitaTras = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorDireitaTras);

        setConversionFactorEncoders((Math.PI * WHEEL_DIAMETER) / CPR);
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivetrainMode(DcMotor.RunMode modo)
    {
        DcMotorEx[] motores = {motorEsquerdaFrente, motorDireitaFrente, motorDireitaTras, motorEsquerdaTras};
        for(DcMotorEx motor : motores)
        {
            motor.setMode(modo);
        }
    }

    public void setConversionFactorEncoders(double factor)
    {
        factorConversionEncoder = factor;
    }

    public void drive(double gamepadLY, double gamepadLX, double gamepadRX)
    {
        drive(gamepadLY, gamepadLX, gamepadRX, false);
    }

    public void drive(double gamepadLY, double gamepadLX, double gamepadRX, boolean fieldOriented)
    {
        double denominator = Math.max(Math.abs(gamepadLY) + Math.abs(gamepadLX) + Math.abs(gamepadRX), 1);
        double frontLeftPower = (gamepadLY + gamepadLX + gamepadRX) / denominator;
        double backLeftPower = (gamepadLY - gamepadLX + gamepadRX) / denominator;
        double frontRightPower = (gamepadLY - gamepadLX - gamepadRX) / denominator;
        double backRightPower = (gamepadLY + gamepadLX - gamepadRX) / denominator;

        motorEsquerdaFrente.setPower(frontLeftPower);
        motorDireitaFrente.setPower(frontRightPower);
        motorEsquerdaTras.setPower(backLeftPower);
        motorDireitaTras.setPower(backRightPower);
    }

    public double getLinearDistance()
    {
        return ((motorEsquerdaFrente.getCurrentPosition() + motorDireitaFrente.getCurrentPosition()) * factorConversionEncoder) / 2;
    }

    public void setLinearTarget(double target)
    {
        DcMotorEx[] motores = {motorEsquerdaFrente, motorDireitaFrente, motorDireitaTras, motorEsquerdaTras};
        for(DcMotorEx motor : motores)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + (int)(target / factorConversionEncoder));
        }
    }

    public boolean atTarget()
    {
        return motorDireitaFrente.isBusy();
    }

    public void setPIDF(double p, double i, double d, double f)
    {
        PIDFCoefficients pidfNew = new PIDFCoefficients(p, i, d, f);
        DcMotorEx[] motores = {motorEsquerdaFrente, motorDireitaFrente, motorDireitaTras, motorEsquerdaTras};
        DcMotorControllerEx[] motorControllerEx = new DcMotorControllerEx[4];
        for(int count = 0; count < motores.length; count++)
        {
            motorControllerEx[count] = (DcMotorControllerEx) motores[count].getController();
            motorControllerEx[count].setPIDFCoefficients(motores[count].getPortNumber(), DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        }
    }
}
