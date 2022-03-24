package frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Paths.FirstPartPath;
import frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto.ShootFromStartingPosition;

public class FirstPart extends ParallelDeadlineGroup {

    public FirstPart() {

        super(

                new FirstPartPath(),
                new ShootFromStartingPosition()

        );
    }

}
