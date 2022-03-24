package frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.chassis.autonomous.shooting.FeederSetForNSeconds;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionStraight;
import frc.robot.commands.intake.IntakeAbsorbAndExtend;
import frc.robot.commands.rollers.RollersAbsorb;
import frc.robot.commands.rollers.RollersStop;
import frc.robot.commands.turret.DriveTurretToAngle;

public class FirstPartPath extends SequentialCommandGroup {

    public FirstPartPath() {
        super(
                new IntakeAbsorbAndExtend(),
                new SpeedReductionStraight(1.04, 0.08, 90),
                new SpeedReductionStraight(-.20, 0.08, 90),
                new DriveTurretToAngle(-10),
                new WaitCommand(2),
                new RollersAbsorb(),
                new FeederSetForNSeconds(2),
                new RollersStop());
    }

}
