// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.turret.Turret;

public class DriveTurretToRight extends CommandBase {


    Turret turret;
    /**
     * Creates a new DriveTurretManually.
     */
    public DriveTurretToRight() {
        // Use addRequirements() here to declare subsystem dependencies.

        turret = Robot.getRobotContainer().getTurret();


        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        turret.setTurretMoveRight();


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        turret.setTurretRaw(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
