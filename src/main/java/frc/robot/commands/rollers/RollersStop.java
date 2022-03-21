package frc.robot.commands.rollers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RollersStop extends CommandBase {

    public RollersStop() {
        addRequirements(Robot.getRobotContainer().getRollers());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        Robot.getRobotContainer().getRollers().setRaw(0);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
