package frc.robot.commands.misc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

public class CopilotRumbleAllOff extends CommandBase {

    /**
     * turns off all rumble from OI's pilot.
     */
    public CopilotRumbleAllOff() {


    }

    @Override
    public void initialize() {
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
