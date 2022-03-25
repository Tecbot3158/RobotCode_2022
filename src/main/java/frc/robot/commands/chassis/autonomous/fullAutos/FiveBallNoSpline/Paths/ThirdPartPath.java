package frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto.ShootFromTerminal;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionStraight;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionTurn;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.rollers.RollersAbsorb;
import frc.robot.commands.turret.DriveTurretToAngle;

public class ThirdPartPath extends SequentialCommandGroup {

    public ThirdPartPath() {
        super(

                new SpeedReductionStraight(2.9, 0.5, 168),
                new DriveTurretToAngle(25),
                // previously 15 deg
                new RollersAbsorb(),
                new FeederSetToSpeed()

        );
    }

}
