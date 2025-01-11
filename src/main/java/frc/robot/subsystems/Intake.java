// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DashboardConstants.IntakeKeys;
import frc.robot.Constants.IntakeConstants.PIDConstants;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Intake extends SubsystemBase
{
    private final SparkMax largeRollerMotor = new SparkMax(IntakeConstants.largeRollerMotorChannel,
        MotorType.kBrushless);

    private final SparkMax smallRollerMotor = new SparkMax(IntakeConstants.smallRollerMotorChannel,
        MotorType.kBrushless);

        private final AnalogInput noteSensor = new AnalogInput(IntakeConstants.opticalSensorPort);
    
    private final XboxController gamepad;

    /** Creates a new Intake. */
    public Intake(XboxController gamepad)
    {
        // Apply configuration common to both large and small roller motors:
        for (SparkMax motor : new SparkMax[] { largeRollerMotor, smallRollerMotor })
        {
            SparkMaxConfig config = new SparkMaxConfig();

            config
                .inverted(motor == largeRollerMotor ? IntakeConstants.isLargeRollerMotorInverted : IntakeConstants.isSmallRollerMotorInverted)
                .idleMode(IntakeConstants.motorIdleMode);

            config.closedLoop
                .pidf(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD, PIDConstants.kFF)
                .iZone(PIDConstants.kIZ)
                .outputRange(PIDConstants.minOutput, PIDConstants.maxOutput);

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            motor.clearFaults();
        }

        

        this.gamepad = gamepad;
    }

    public void intake()
    {
        largeRollerMotor.getClosedLoopController()
            .setReference(
                SmartDashboard.getNumber(IntakeKeys.largeRollerIntakeRpm, IntakeConstants.largeRollerMotorIntakeRpm),
                SparkBase.ControlType.kVelocity);

        smallRollerMotor.getClosedLoopController()
            .setReference(
                SmartDashboard.getNumber(IntakeKeys.smallRollerIntakeRpm, IntakeConstants.smallRollerMotorIntakeRpm),
                SparkBase.ControlType.kVelocity);
    }

    public void eject()
    {
        largeRollerMotor.getClosedLoopController()
            .setReference(
                SmartDashboard.getNumber(IntakeKeys.largeRollerEjectRpm, IntakeConstants.largeRollerMotorEjectRpm),
                SparkBase.ControlType.kVelocity);

        smallRollerMotor.getClosedLoopController()
            .setReference(
                SmartDashboard.getNumber(IntakeKeys.smallRollerEjectRpm, IntakeConstants.smallRollerMotorEjectRpm),
                SparkBase.ControlType.kVelocity);
    }

    public void stop()
    {
        largeRollerMotor.stopMotor();
        smallRollerMotor.stopMotor();
    }

    public boolean hasNote()
    {
        var voltage = noteSensor.getVoltage();
        SmartDashboard.putNumber(IntakeKeys.noteSensorVoltage, voltage);
        return voltage > IntakeConstants.noteSensorVoltageThreshold;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        var hasNote = hasNote();
        gamepad.setRumble(RumbleType.kLeftRumble, hasNote ? 1 : 0);
        SmartDashboard.putBoolean(IntakeKeys.hasNote, hasNote);
        SmartDashboard.putNumber("Intake/Large/RPM", largeRollerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake/Small/RPM", smallRollerMotor.getEncoder().getVelocity());

    }
}
