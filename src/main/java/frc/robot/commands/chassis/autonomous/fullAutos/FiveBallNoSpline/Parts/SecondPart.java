package frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Paths.FirstPartPath;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Paths.SecondPath;
import frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto.ShootFromStartingPosition;
import frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto.ShootFromThirdBall;

public class SecondPart extends ParallelRaceGroup {

    public SecondPart() {
        super(

                new SecondPath(),
                new ShootFromThirdBall()

        );
    }

}
