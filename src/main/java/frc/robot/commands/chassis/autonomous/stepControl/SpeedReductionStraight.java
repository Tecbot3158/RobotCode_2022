/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis.autonomous.stepControl;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SpeedReductionStraight extends CommandBase {
    /**
     * Moves straight using speed reduction control mode.
     */
    double meters, target, maxPower, angle;

    double CONFIG_NOT_SET = -15069;

    public SpeedReductionStraight(double meters, double maxPower) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        this.meters = meters;
        this.maxPower = maxPower;
        angle = CONFIG_NOT_SET;
    }

    public SpeedReductionStraight(double meters, double maxPower, double angle) {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        this.meters = meters;
        this.maxPower = maxPower;
        this.angle = angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        target = Robot.getRobotContainer().getDriveTrain().getLeftDistance() + meters;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.getRobotContainer().getDriveTrain().stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (angle != CONFIG_NOT_SET) {
            return Robot.getRobotContainer().getDriveTrain().moveStraight(target, maxPower, angle);
        } else {
            return Robot.getRobotContainer().getDriveTrain().moveStraight(target, maxPower);
        }
    }
}
