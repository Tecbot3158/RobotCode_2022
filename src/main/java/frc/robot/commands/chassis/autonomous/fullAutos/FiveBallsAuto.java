package frc.robot.commands.chassis.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.turret.MoveTurretToAngle;
import frc.robot.resources.splines.PieceWiseSpline;

public class FiveBallsAuto extends ParallelCommandGroup {

    public FiveBallsAuto(PieceWiseSpline fiveBallAutoPath) {

        super(
                new MoveTurretToAngle(-180),
                new FiveBallsPath(fiveBallAutoPath),
                new EverythingToShoot());

    }

}
