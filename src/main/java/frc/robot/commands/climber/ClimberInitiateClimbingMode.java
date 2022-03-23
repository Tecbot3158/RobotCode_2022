package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.intake.basic.IntakeExtend;
import frc.robot.commands.turret.DriveTurretToAngle;

public class ClimberInitiateClimbingMode extends ParallelCommandGroup {
    public ClimberInitiateClimbingMode() {
        super(
                new IntakeExtend(),
                new ChassisSetDefaultDrive(),
                new DriveTurretToAngle(0)
        );
    }
}
