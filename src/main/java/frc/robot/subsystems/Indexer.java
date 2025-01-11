// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.DashboardConstants.IndexerKeys;
import frc.robot.Constants.IndexerConstants.UpperPIDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.IndexerConstants.LowerPIDConstants;

public class Indexer extends SubsystemBase
{
    private final SparkMax lowerRollerMotor = new SparkMax(IndexerConstants.lowerRollerMotorChannel,
        MotorType.kBrushless);

    private final SparkMax upperRollerMotor = new SparkMax(IndexerConstants.upperRollerMotorChannel,
        MotorType.kBrushless);

    /** Creates a new Indexer. */
    public Indexer()
    {
        // Apply configuration common to both large and small roller motors:
        for (SparkMax motor : new SparkMax[] { lowerRollerMotor, upperRollerMotor })
        {
            SparkMaxConfig config = new SparkMaxConfig();
            
            config
                .idleMode(IndexerConstants.motorIdleMode);

            if (motor == lowerRollerMotor)
            {
                config
                    .inverted(IndexerConstants.isLowerRollerMotorInverted);
                
                config.closedLoop
                    .pidf(LowerPIDConstants.kP, LowerPIDConstants.kI, LowerPIDConstants.kD, LowerPIDConstants.kFF)
                    .iZone(LowerPIDConstants.kIZ)
                    .outputRange(LowerPIDConstants.minOutput, LowerPIDConstants.maxOutput);
            }
            else
            {
                config
                    .inverted(IndexerConstants.isUpperRollerMotorInverted);
                
                config.closedLoop
                    .pidf(UpperPIDConstants.kP, UpperPIDConstants.kI, UpperPIDConstants.kD, UpperPIDConstants.kFF)
                    .iZone(UpperPIDConstants.kIZ)
                    .outputRange(UpperPIDConstants.minOutput, UpperPIDConstants.maxOutput);
            }

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            motor.clearFaults();
        }
    }

    public void intake()
    {
        lowerRollerMotor.getClosedLoopController()
            .setReference(SmartDashboard.getNumber(IndexerKeys.lowerRollerIntakeRpm,
                IndexerConstants.lowerRollerMotorIntakeRpm),
                SparkBase.ControlType.kVelocity);

        upperRollerMotor.getClosedLoopController()
            .setReference(SmartDashboard.getNumber(IndexerKeys.upperRollerIntakeRpm,
                IndexerConstants.upperRollerMotorIntakeRpm),
                SparkBase.ControlType.kVelocity);
    }

    public void eject()
    {
        lowerRollerMotor.getClosedLoopController()
            .setReference(SmartDashboard.getNumber(IndexerKeys.lowerRollerEjectRpm,
                IndexerConstants.lowerRollerMotorEjectRpm),
                SparkBase.ControlType.kVelocity);

        upperRollerMotor.getClosedLoopController()
            .setReference(SmartDashboard.getNumber(IndexerKeys.upperRollerEjectRpm,
                IndexerConstants.upperRollerMotorEjectRpm),
                SparkBase.ControlType.kVelocity);
    }

    public void feed()
    {
        lowerRollerMotor.getClosedLoopController()
            .setReference(SmartDashboard.getNumber(IndexerKeys.lowerRollerFeedRpm,
                IndexerConstants.lowerRollerMotorFeedRpm),
                SparkBase.ControlType.kVelocity);
        upperRollerMotor.getClosedLoopController()
            .setReference(SmartDashboard.getNumber(IndexerKeys.upperRollerFeedRpm,
                IndexerConstants.upperRollerMotorFeedRpm),
                SparkBase.ControlType.kVelocity);
    }

    public void stop()
    {
        lowerRollerMotor.stopMotor();
        upperRollerMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Indexer/Lower/RPM", lowerRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Indexer/Upper/RPM", upperRollerMotor.getEncoder().getVelocity());
    }
}
