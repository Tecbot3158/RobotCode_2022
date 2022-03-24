package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.climber.basic.ClimberSetRawMotors;
import frc.robot.commands.intake.basic.IntakeExtend;
import frc.robot.commands.turret.DriveTurretToAngle;

public class ClimberInitiateClimbingMode extends SequentialCommandGroup {
    public ClimberInitiateClimbingMode() {
        super(
                new IntakeExtend(),
                new ChassisSetDefaultDrive(),
                new DriveTurretToAngle(0));
    }
}
