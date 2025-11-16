// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.Shooter;

/**
 * Tests the Shooter subsystem.
 */
public class TestShooter extends SequentialCommandGroup
{
    public TestShooter(Shooter shooter)
    {
        addCommands(
            new InstantCommand(() -> System.out.println("Testing Shooter: Amp")),
            new TestShooterAmp(shooter).withTimeout(TestConstants.shooterMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs),

            new InstantCommand(() -> System.out.println("Testing Shooter: Speaker")),
            new TestShooterSpeaker(shooter).withTimeout(TestConstants.shooterMotorTimeoutSecs),
            new WaitCommand(TestConstants.inbetweenTimeSecs)
        );
    }
}
