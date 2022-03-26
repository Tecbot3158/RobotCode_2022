package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.misc.CopilotRumbleSetForTime;
import frc.robot.subsystems.climber.Climber;

public class ClimberToggleFreeMode extends CommandBase {


    public ClimberToggleFreeMode() {

    }

    @Override
    public void initialize() {

        Robot.debug("INITTT FREEE ");

        if (Climber.isFreeMode()) {
            // if in free mode,
            // go to capped encoder mode
            Climber.setFreeMode(false);
            new CopilotRumbleSetForTime(0.1, 0.2).schedule();
            Robot.debug("FREE MODE: NEEEIN");

        } else {
            // if not in free mode
            // then go to FREE mode !

            Climber.setFreeMode(true);
            new CopilotRumbleSetForTime(0.6, 0.2).schedule();
            Robot.debug("FREE MODE: OUIIIIii");
        }

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
