package frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts.FirstPart;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts.SecondPart;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts.ThirdPart;

public class FiveBallsNoSpline extends SequentialCommandGroup {

    public FiveBallsNoSpline() {
        // super(
        // new IntakeAbsorbAndExtend(),
        // new ShootFromStartingPosition(),
        // new SpeedReductionStraight(1.04, 0.4, 90),
        // new SpeedReductionStraight(-1.04, 0.4, 90),
        // new RollersAbsorb(),
        // new FeederSetForNSeconds(5),
        // new RollersStop(),
        // new ShootFromThirdBall(),
        // new SpeedReductionTurn(180, 0.5),
        // new SpeedReductionStraight(2.2606, 0.4, 180),
        // new DriveTurretToAngleRelativeToRobot(-45),
        // new RollersAbsorb(),
        // new FeederSetForNSeconds(3),
        // new RollersStop(),
        // new ShootFromTerminal(),
        // new SpeedReductionTurn(172.7, 0.5),
        // new SpeedReductionStraight(6.096, 0.4, 172.7),
        // new DriveTurretToAngleRelativeToRobot(-15),
        // new RollersAbsorb(),
        // new FeederSetToSpeed()
        //
        // );

        super(new FirstPart(),
                new SecondPart(),
                new ThirdPart());
    }

}
