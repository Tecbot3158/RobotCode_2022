package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.OI;
import frc.robot.commands.climber.basic.ClimberSetRawMotors;

public class ClimberSetInitThenClearBindingsThenSetClimberBindings extends CommandBase {

    /**
     * Clears ALL BUTTON bindings through the Command Scheduler,
     * when initialized.
     */
    public ClimberSetInitThenClearBindingsThenSetClimberBindings() {
    }

    @Override
    public void initialize() {

        new ClimberInitiateClimbingMode().schedule();

        CommandScheduler.getInstance().clearButtons();

        OI.getInstance().configureButtonBindingsForClimbingMode();

        new ClimberSetRawMotors().schedule( );
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
