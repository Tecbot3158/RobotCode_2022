package frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterShootCustomTarger;

public class ShootFromStartingPosition extends SequentialCommandGroup {

    public ShootFromStartingPosition() {
        super(new ShooterShootCustomTarger(3500));
    }

}
