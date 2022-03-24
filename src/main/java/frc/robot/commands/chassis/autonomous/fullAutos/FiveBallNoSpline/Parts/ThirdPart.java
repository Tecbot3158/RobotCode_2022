package frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Paths.ThirdPartPath;
import frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto.ShootFromTerminal;

public class ThirdPart extends ParallelRaceGroup {

    public ThirdPart() {
        super(
                new ThirdPartPath(),
                new ShootFromTerminal());

    }

}
