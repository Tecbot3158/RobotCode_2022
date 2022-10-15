// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.drivingModes;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.chassis.DriveTrain;

public class ToggleDragonFlyRaw extends InstantCommand {
    DriveTrain driveTrain;

    /** Creates a new ToggleDragonFlyWheel. */
    public ToggleDragonFlyRaw() {
        driveTrain = Robot.getRobotContainer().getDriveTrain();
        addRequirements(driveTrain);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("TOGGLING DRAGONFLYWHEEL");
        System.out.println("Valor antes: " + driveTrain.getDragonFlySolenoidValue());

        if (driveTrain.getDragonFlySolenoidValue() == DoubleSolenoid.Value.kForward) {
            driveTrain.setDragonFlyRaw(DoubleSolenoid.Value.kReverse);
        } else {
            driveTrain.setDragonFlyRaw(DoubleSolenoid.Value.kForward);
        }

        System.out.println("Valor después: " + driveTrain.getDragonFlySolenoidValue());
    }
}
