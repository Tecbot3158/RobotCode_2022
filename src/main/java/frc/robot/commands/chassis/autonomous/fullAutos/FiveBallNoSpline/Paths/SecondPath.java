package frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.autonomous.shooting.FeederSetForNSeconds;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionStraight;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionTurn;
import frc.robot.commands.rollers.RollersAbsorb;
import frc.robot.commands.rollers.RollersStop;
import frc.robot.commands.turret.DriveTurretToAngle;

public class SecondPath extends SequentialCommandGroup {

    public SecondPath() {
        super(
                new SpeedReductionTurn(180, 0.5),
                new SpeedReductionStraight(2.2606, 0.4, 180),
                new DriveTurretToAngle(-45),
                new RollersAbsorb(),
                new FeederSetForNSeconds(3),
                new RollersStop());
    }

}
