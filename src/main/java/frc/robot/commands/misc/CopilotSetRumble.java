package frc.robot.commands.misc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

public class CopilotSetRumble extends CommandBase {

    double value;

    /**
     * turns off all rumble from OI's pilot.
     */
    public CopilotSetRumble( double value ) {
        this.value = value;


    }

    @Override
    public void initialize() {
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble, value);
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble, value);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
