package frc.robot.commands.chassis.autonomous.fullAutos;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.chassis.autonomous.splines.SplineMove;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionStraight;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.intake.basic.IntakeTogglePositionAndMotors;
import frc.robot.commands.rollers.RollersAbsorb;
import frc.robot.commands.rollers.RollersMove;
import frc.robot.commands.shooter.ShooterGoToTarget;
import frc.robot.resources.splines.PieceWiseSpline;

public class FiveBallsPath extends SequentialCommandGroup {

    public FiveBallsPath(PieceWiseSpline fiveBallAutoSpline) {
        super(

                new SpeedReductionStraight(0.9, 0.4, 90),
                new SpeedReductionStraight(-0.9, 0.4, 90),
                new SplineMove(fiveBallAutoSpline, 0.3, true, false, true, false)

        );
    }

}
