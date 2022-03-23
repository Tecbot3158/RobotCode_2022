package frc.robot.commands.chassis.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.chassis.autonomous.splines.SplineMove;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionStraight;
import frc.robot.resources.splines.PieceWiseSpline;

public class FiveBallsPath extends SequentialCommandGroup {

    public FiveBallsPath(PieceWiseSpline fiveBallAutoSpline) {
        super(

                new SpeedReductionStraight(0.9, 0.4, 90),
                new SpeedReductionStraight(-0.9, 0.4, 90),
                new SplineMove(fiveBallAutoSpline, 0.3, true, false, true, false),
                new WaitCommand(5),
                new SpeedReductionStraight(-4.8, 0.4, 180)

        );
    }

}
