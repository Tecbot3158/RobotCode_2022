package frc.robot.commands.chassis.autonomous.fullAutos.TwoBalls;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.chassis.autonomous.shooting.FeederSetForNSeconds;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionNoInitialAngle;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionStraight;
import frc.robot.commands.intake.IntakeAbsorbAndExtend;
import frc.robot.commands.rollers.RollersAbsorb;
import frc.robot.commands.rollers.RollersStop;
import frc.robot.commands.turret.DriveTurretToAngle;

public class FirstPartTwoBallsPath extends SequentialCommandGroup {

    public FirstPartTwoBallsPath() {
        super(
                new IntakeAbsorbAndExtend(),
                new SpeedReductionNoInitialAngle(1.44, 0.08),
                new SpeedReductionNoInitialAngle(-1.5, 0.08),
                new WaitCommand(2),
                new RollersAbsorb(),
                new FeederSetForNSeconds(2),
                new RollersStop());
    }

}
