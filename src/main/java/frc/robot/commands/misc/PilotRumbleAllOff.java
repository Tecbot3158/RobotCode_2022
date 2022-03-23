package frc.robot.commands.misc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.OI;
import frc.robot.commands.intake.basic.IntakeAbsorb;
import frc.robot.commands.intake.basic.IntakeExtend;

public class PilotRumbleAllOff extends CommandBase {

    /**
     * turns off all rumble from OI's pilot.
     */
    public PilotRumbleAllOff() {


    }

    @Override
    public void initialize() {
        OI.getInstance().getPilot().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        OI.getInstance().getPilot().setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
