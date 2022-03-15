package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.basic.IntakeAbsorb;
import frc.robot.commands.intake.basic.IntakeExtend;

public class IntakeAbsorbAndExtend extends ParallelCommandGroup {

    public IntakeAbsorbAndExtend(){
        addCommands( new IntakeAbsorb(),
                  new IntakeExtend());
    }

}
