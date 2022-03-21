package frc.robot.commands.chassis.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.intake.IntakeAbsorbAndExtend;
import frc.robot.commands.rollers.RollersAbsorb;
import frc.robot.commands.shooter.ShooterGoToTarget;

public class EverythingToShoot extends ParallelCommandGroup {

    public EverythingToShoot() {

        super(

                new IntakeAbsorbAndExtend(),
                new RollersAbsorb(),
                new FeederSetToSpeed(),
                new ShooterGoToTarget()

        );
    }

}
