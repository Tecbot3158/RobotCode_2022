// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.TecbotCamera;

public class MoveTurretToTarget extends CommandBase {

    TecbotCamera vision;
    Turret turret;

    /**
     * Creates a new MoveTurretToTarget.
     */
    public MoveTurretToTarget() {
        // Use addRequirements() here to declare subsystem dependencies.

        vision = Robot.getRobotContainer().getTecbotCamera();
        turret = Robot.getRobotContainer().getTurret();

        addRequirements(turret, vision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        vision.update();
        double yaw = Robot.getRobotContainer().getTecbotCamera().getYaw();
        // turret.settoTarget(yaw);

        // if (turret.getTurretPID().atSetpoint()) {
        //
        // end(true);
        //
        // }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
