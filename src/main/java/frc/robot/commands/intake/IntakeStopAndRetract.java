package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.basic.IntakeRetract;
import frc.robot.commands.intake.basic.IntakeStopMotors;

public class IntakeStopAndRetract extends ParallelCommandGroup {
    public IntakeStopAndRetract(){
       addCommands( new IntakeStopMotors(),
                    new IntakeRetract() );
    }
}
