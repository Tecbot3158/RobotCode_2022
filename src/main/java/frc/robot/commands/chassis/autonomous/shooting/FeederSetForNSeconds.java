package frc.robot.commands.chassis.autonomous.shooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.feeder.FeederStop;

public class FeederSetForNSeconds extends SequentialCommandGroup {

    public FeederSetForNSeconds(double seconds) {

        super(

                new FeederSetToSpeed(),
                new WaitCommand(seconds),
                new FeederStop()

        );

    }

}
