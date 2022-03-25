package frc.robot.commands.chassis.autonomous.fullAutos.TwoBalls;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.autonomous.shooting.FiveBallsAuto.ShootFromStartingPosition;

public class TwoBallsAuto extends ParallelCommandGroup {

    public TwoBallsAuto() {
        super(new ShootFromStartingPosition(),
                new FirstPartTwoBallsPath());
    }

}
