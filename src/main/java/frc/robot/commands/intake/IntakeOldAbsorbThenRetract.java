package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class IntakeOldAbsorbThenRetract extends CommandBase {

    Intake intake;

    /**
     * Creates a new DoIntake.
     */
    public IntakeOldAbsorbThenRetract() {
        // Use addRequirements() here to declare subsystem dependencies.

        intake = Robot.getRobotContainer().getIntake();
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.intakeExtend();
        intake.intakeMotorsAbsorb();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intakeRetract();
        intake.intakeMotorsOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
