package frc.robot.commands.chassis.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.turret.DriveTurretToAngleRelativeToRobot;

public class FiveBallsNoSpline extends SequentialCommandGroup {

    public FiveBallsNoSpline() {
        super(
                new IntakeAbsorbAndExtend(),
                new ShootFromStartingPosition(),
                new SpeedReductionStraight(1.04, 0.4, 90),
                new SpeedReductionStraight(-1.04, 0.4, 90),
                new RollersAbsorb(),
                new FeederSetForNSeconds(5),
                new RollersStop(),
                new ShootFromThirdBall(),
                new SpeedReductionTurn(180, 0.5),
                new SpeedReductionStraight(2.2606, 0.4, 180),
                new DriveTurretToAngleRelativeToRobot(-45),
                new RollersAbsorb(),
                new FeederSetForNSeconds(3),
                new RollersStop(),
                new ShootFromTerminal(),
                new SpeedReductionTurn(172.7, 0.5),
                new SpeedReductionStraight(6.096, 0.4, 172.7),
                new DriveTurretToAngleRelativeToRobot(-15),
                new RollersAbsorb(),
                new FeederSetToSpeed()

        );
    }

}
