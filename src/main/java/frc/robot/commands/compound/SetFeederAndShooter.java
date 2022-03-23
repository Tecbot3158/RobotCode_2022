package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.shooter.ShooterGoToTarget;

public class SetFeederAndShooter  extends ParallelCommandGroup {

    public SetFeederAndShooter(){
        addCommands( new FeederSetToSpeed(), new ShooterGoToTarget());

    }

}
