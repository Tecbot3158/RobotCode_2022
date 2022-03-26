package frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterShootCustomTarger;

public class ShootFromTerminal extends SequentialCommandGroup {

    public ShootFromTerminal() {
        super(new ShooterShootCustomTarger(5400 ));
        // before was 5250
    }

}
