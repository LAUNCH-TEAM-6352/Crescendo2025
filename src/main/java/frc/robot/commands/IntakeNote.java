package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * IntakeNote intakes a note from the floor
 * move manipulator into intake position
 * start indexer running for intake
 * tell Intake subsystem to intake
 */
public class IntakeNote extends Command
{
    private final Intake intake;
    private final Indexer indexer;
    private String timeoutKey = null;
    private long stopTime;

    /**
     * 
     * 
     * @param intake
     */
    public IntakeNote(Intake intake, Indexer indexer)
    {
        this.intake = intake;
        this.indexer = indexer;
        // Specify required subsystems
        addRequirements(intake, indexer);
    }

    public IntakeNote(Intake intake, Indexer indexer, String timeoutKey)
    {
        this(intake , indexer);
        this.timeoutKey = timeoutKey;
    }
    
    @Override
    public void initialize()
    {
        indexer.intake();
        intake.intake();

        stopTime = timeoutKey == null
            ? Long.MAX_VALUE
            : RobotController.getFPGATime() + (long) (Constants.microsecondsPerSecond * SmartDashboard.getNumber(timeoutKey, 180));
    }

    @Override
    public void execute()
    {
        // Intentionally blank, PID Controllers are controlling motors
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted)
    {
        intake.stop();
       indexer.stop();
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished()
    {
        return RobotController.getFPGATime() >= stopTime || intake.hasNote();
    }
}
