package frc.robot.commands.misc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.OI;

public class CopilotRumbleSetForTime extends SequentialCommandGroup {

    /**
     * turns off all rumble from OI's pilot.
     */
    public CopilotRumbleSetForTime( double rumbleValue, double time) {
        addCommands(new CopilotSetRumble(rumbleValue));
        addCommands(new WaitCommand(time));
        addCommands(new CopilotRumbleAllOff());

    }

}
