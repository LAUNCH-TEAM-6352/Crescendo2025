// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DashboardConstants.ShooterKeys;
import frc.robot.Constants.ShooterConstants.PIDConstants;

public class Shooter extends SubsystemBase
{
    private final SparkMax leftMotor = new SparkMax(ShooterConstants.leftMotorChannel,
        MotorType.kBrushless);

    private final SparkMax rightMotor = new SparkMax(ShooterConstants.rightMotorChannel,
        MotorType.kBrushless);

    private final SparkMax[] motors = new SparkMax[] { leftMotor, rightMotor };

    private boolean isSpinningUp = false;
    private boolean isAtTargetVelocity = false;

    private double targetVelocity;

    private double lastLeftVelocity;
    private double lastRightVelocity;

    private double velocityTolerance;

    private long startTime;

    /** Creates a new Shooter. */
    public Shooter()
    {
        // Apply configuration common to both large and small roller motors:
        for (SparkMax motor : motors)
        {
            SparkMaxConfig config = new SparkMaxConfig();
            
            config
                .inverted(motor == leftMotor ? ShooterConstants.isLeftMotorInverted : ShooterConstants.isRightMotorInverted)
                .idleMode(ShooterConstants.motorIdleMode);

            config.closedLoop
                .pidf(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD, PIDConstants.kFF)
                .iZone(PIDConstants.kIZ)
                .outputRange(PIDConstants.minOutput, PIDConstants.maxOutput);

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            motor.clearFaults();
        }
    }

    public void setAmpSpeed()
    {
        setVelocity(SmartDashboard.getNumber(ShooterKeys.ampRpm, ShooterConstants.ampRpm));
    }

    public void setSpeakerSpeed()
    {
        setVelocity(SmartDashboard.getNumber(ShooterKeys.speakerRpm, ShooterConstants.speakerRpm));
    }

    /**
     * Sets both motors to run at the specified velocity (in RPM).
     */
    private void setVelocity(double velocity)
    {
        targetVelocity = velocity;
        velocityTolerance = SmartDashboard.getNumber(ShooterKeys.rpmTolerance, ShooterConstants.rpmTolerance);

        lastLeftVelocity = 0;
        lastRightVelocity = 0;

        for (SparkMax motor : motors)
        {
            motor.getClosedLoopController().setReference(targetVelocity, SparkBase.ControlType.kVelocity);
        }

        isSpinningUp = true;
        isAtTargetVelocity = false;

        startTime = RobotController.getFPGATime();
    }

    public void stop()
    {
        for (SparkMax motor : motors)
        {
            motor.stopMotor();
        }
    }

    public boolean isAtTargetVelocity()
    {
        return isAtTargetVelocity;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        double leftVelocity = leftMotor.getEncoder().getVelocity();
        double rightVelocity = rightMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("Shooter/Left/RPM", leftVelocity);
        SmartDashboard.putNumber("Shooter/Right/RPM", rightVelocity);

        // Determine if both shooter motors have come up to speed and stabalized:
        if (isSpinningUp)
        {
            if ((Math.abs(leftVelocity - targetVelocity) < velocityTolerance &&
                Math.abs(leftVelocity - lastLeftVelocity) < velocityTolerance &&
                Math.abs(rightVelocity - targetVelocity) < velocityTolerance &&
                Math.abs(rightVelocity - lastRightVelocity) < velocityTolerance) ||
                RobotController.getFPGATime() - startTime > 3000000
                )
            {
                isSpinningUp = false;
                isAtTargetVelocity = true;
            }
            else
            {
                lastLeftVelocity = leftVelocity;
                lastRightVelocity = rightVelocity;
            }
        }
    }
}
