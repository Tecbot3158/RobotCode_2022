package frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.autonomous.shooting.FeederSetForNSeconds;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionStraight;
import frc.robot.commands.intake.IntakeAbsorbAndExtend;
import frc.robot.commands.rollers.RollersAbsorb;
import frc.robot.commands.rollers.RollersStop;

public class FirstPartPath extends SequentialCommandGroup {

    public FirstPartPath() {
        super(
                new IntakeAbsorbAndExtend(),
                new SpeedReductionStraight(1.04, 0.4, 90),
                new SpeedReductionStraight(-1.04, 0.4, 90),
                new RollersAbsorb(),
                new FeederSetForNSeconds(5),
                new RollersStop());
    }

}
