package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Mecanum {
    public static final double MAX_VELOCITY = 12; // in/s
    private static final double CPR = 1680;
    private static final double WHEEL_DIAMETER = 4;

    private final DcMotorEx motorEsquerdaFrente;
    private final DcMotorEx motorDireitaFrente;
    private final DcMotorEx motorEsquerdaTras;
    private final DcMotorEx motorDireitaTras;
    private final IMU imu;

    private double factorConversionEncoder = 1;

    public Mecanum(OpMode opMode) {
        motorEsquerdaFrente = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorEsquerdaFrente);
        motorDireitaFrente = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorDireitaFrente);
        motorEsquerdaTras = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorEsquerdaTras);
        motorDireitaTras = opMode.hardwareMap.get(DcMotorEx.class, Constants.DriveTrainMotorsNames.motorDireitaTras);

        motorEsquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = opMode.hardwareMap.get(IMU.class, "imu");

        setConversionFactorEncoders(WHEEL_DIAMETER * Math.PI / CPR);
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
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

    public double getLinearDistanceOfOneMotor()
    {
        return motorEsquerdaFrente.getCurrentPosition() * factorConversionEncoder;
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
        return motorEsquerdaFrente.isBusy();
    }

    public void setPIDFForPosition(double p, double i, double d)
    {
        PIDFCoefficients pidfNew = new PIDFCoefficients(p, i, d, 0);
        DcMotorEx[] motores = {motorEsquerdaFrente, motorDireitaFrente, motorDireitaTras, motorEsquerdaTras};
        DcMotorControllerEx[] motorControllerEx = new DcMotorControllerEx[4];
        for(int count = 0; count < motores.length; count++)
        {
            motorControllerEx[count] = (DcMotorControllerEx) motores[count].getController();
            motorControllerEx[count].setPIDFCoefficients(motores[count].getPortNumber(), DcMotor.RunMode.RUN_TO_POSITION, pidfNew);
        }
    }

    public void setPIDFForVelocity(double p, double i, double d, double f)
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

    public double getHeading()
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
