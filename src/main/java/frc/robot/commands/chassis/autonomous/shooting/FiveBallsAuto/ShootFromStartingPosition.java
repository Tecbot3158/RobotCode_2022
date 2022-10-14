package frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterShootCustomTarger;

public class ShootFromStartingPosition extends SequentialCommandGroup {

    public ShootFromStartingPosition() {
        // super(new ShooterShootCustomTarger(3800));
        // super(new ShooterShootCustomTarger(4180));
        // super(new ShooterShootCustomTarger(4598));
        super(new ShooterShootCustomTarger(4700));
    }

}

// poner 3 segundos de tiempo y que se pare
// la partd de enfrente este alineada con la pelta y la parte de atras cin el
// hub, tobot tocando la linea del tarmac, liea exterior