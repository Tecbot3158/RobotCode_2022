package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.feeder.FeederRaw;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.shooter.ShooterGoRaw;
import frc.robot.commands.shooter.ShooterGoToTarget;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Feeder;

public class SetFeederAndShooter  extends ParallelCommandGroup {

    public SetFeederAndShooter(){
        addCommands( new FeederSetToSpeed(), new ShooterGoToTarget());

    }

}
