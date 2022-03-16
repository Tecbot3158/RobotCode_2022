package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class IntakeAbsorbExtendThenRetractStop extends StartEndCommand {
    /**
     * Creates a new StartEndCommand. Will run the given runnables when the command starts and when it
     * ends.
     *
     * @param onInit       the Runnable to run on command init
     * @param onEnd        the Runnable to run on command end
     * @param requirements the subsystems required by this command
     */
    public IntakeAbsorbExtendThenRetractStop(Runnable onInit, Runnable onEnd, Subsystem... requirements) {
        super(onInit, onEnd, requirements);
    }

    //   public IntakeAbsorbExtendThenRetractStop(){ //       Intake intake = Robot.getRobotContainer().getIntake(); //       super( new IntakeAbsorbAndExtend(), new IntakeStopAndRetract(), null ); // //   } // }
}