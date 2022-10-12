/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DefaultDrive extends CommandBase {
    /**
     * Creates a new Command.
     */
    public DefaultDrive() {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // left y
        // double x = Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1,
        // 1) * 0.85;
        // double x = Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1,
        // 1) ;
        // ponce down
        // double x = Math.clamp(-(OI.getInstance().getPilot().getRightAxisX(false)),
        // -1, 1);
        // left x
        // double y = -Math.clamp((OI.getInstance().getPilot().getLeftAxisY()), -1, 1);
        // double y = -Math.clamp((OI.getInstance().getPilot().getLeftAxisY()), -1, 1);
        // double y = Math.clamp((OI.getInstance().getPilot().getTriggers()), -1, 1);

        double x = OI.getInstance().getDefaultDriveInputX() * RobotMap.DRIVE_TRAIN_INPUT_FACTOR;
        double y = OI.getInstance().getDefaultDriveInputY() * RobotMap.DRIVE_TRAIN_INPUT_FACTOR;

        // right x
        double turn = (OI.getInstance().getPilot().getRightAxisX()) * RobotMap.DRIVE_TRAIN_INPUT_FACTOR;
        // Triggers
        double middleWheel = -OI.getInstance().getMiddleWheel() * RobotMap.DRIVE_TRAIN_INPUT_FACTOR;
        // double middleWheel = 0;
        SmartDashboard.putNumber("TRIGGERS ", middleWheel);

        Robot.getRobotContainer().getDriveTrain().defaultDrive(x, y, turn, middleWheel);
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
