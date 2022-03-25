package frc.robot.commands.chassis.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts.FirstPart;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts.SecondPart;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Paths.FirstPartPath;
import frc.robot.commands.chassis.autonomous.shooting.FeederSetForNSeconds;
import frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto.ShootFromStartingPosition;
import frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto.ShootFromTerminal;
import frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto.ShootFromThirdBall;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionStraight;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionTurn;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.intake.IntakeAbsorbAndExtend;
import frc.robot.commands.rollers.RollersAbsorb;
import frc.robot.commands.rollers.RollersStop;
import frc.robot.commands.turret.DriveTurretToAngle;

public class ThreeBallsAuto extends SequentialCommandGroup {

    public ThreeBallsAuto() {

        super(new FirstPart(),
                new SecondPart());
    }
}
