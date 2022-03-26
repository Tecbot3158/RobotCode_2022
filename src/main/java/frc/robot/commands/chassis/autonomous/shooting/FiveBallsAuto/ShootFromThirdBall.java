package frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterShootCustomTarger;

public class ShootFromThirdBall extends SequentialCommandGroup {

    public ShootFromThirdBall() {
        super(new ShooterShootCustomTarger( 4000 ));
        // before was 4300
    }

}
