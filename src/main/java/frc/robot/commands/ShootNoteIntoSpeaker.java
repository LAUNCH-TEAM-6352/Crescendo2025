// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

/**
 * ShootNoteIntoAmp shoots a note into the Amp
 * move manipulator into speaker position
 * start shooter
 * tell indexer subsystem to feed
 * determine what feedback will be provided to drivers during command execution
 */
public class ShootNoteIntoSpeaker extends Command
{
    private final Indexer indexer;
    private final Shooter shooter;
    private String timeoutKey = null;
    private long stopTime;

    private boolean hasShooterBeenFed;

    /**
     *
     * 
     * @param indexer
     */
    public ShootNoteIntoSpeaker(Indexer indexer, Shooter shooter)
    {
        this.indexer = indexer;
        this.shooter = shooter;
        // Specify required subsystems
        addRequirements(indexer, shooter);
    }

    public ShootNoteIntoSpeaker(Indexer indexer, Shooter shooter, String timeoutKey)
    {
        this(indexer, shooter);
        this.timeoutKey = timeoutKey;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        shooter.setSpeakerSpeed();
        hasShooterBeenFed = false;

        stopTime = timeoutKey == null
            ? Long.MAX_VALUE
            : RobotController.getFPGATime() + (long) (Constants.microsecondsPerSecond * SmartDashboard.getNumber(timeoutKey, 180));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if (shooter.isAtTargetVelocity() && !hasShooterBeenFed)
        {
            indexer.feed();
            hasShooterBeenFed = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        indexer.stop();
        shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return RobotController.getFPGATime() >= stopTime;
    }
}
