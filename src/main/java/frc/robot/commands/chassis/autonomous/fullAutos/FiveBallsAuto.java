package frc.robot.commands.chassis.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.resources.splines.PieceWiseSpline;

public class FiveBallsAuto extends ParallelCommandGroup {

    public FiveBallsAuto(PieceWiseSpline fiveBallAutoPath) {

        super(
                // dnew DriveTurretToVisionTarget(),
                new FiveBallsPath(fiveBallAutoPath),
                new EverythingToShoot());

    }

}
