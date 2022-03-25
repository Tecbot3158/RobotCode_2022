package frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts.FirstPart;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts.SecondPart;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.Parts.ThirdPart;

public class FiveBallsNoSpline extends SequentialCommandGroup {

    public FiveBallsNoSpline() {
        super(new FirstPart(),
                new SecondPart(),
                new ThirdPart());
    }

}
