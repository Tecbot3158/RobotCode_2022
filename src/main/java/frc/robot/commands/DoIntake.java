package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class DoIntake extends CommandBase {

    Intake intake;

    /** Creates a new DoIntake. */
    public DoIntake(Intake subsys) {
        // Use addRequirements() here to declare subsystem dependencies.

        intake = subsys;
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.intakeExtend();
        intake.intakeAbsorb();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.intakeRetract();
        intake.intakeOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}