// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.turret.Turret;

public class MoveTurretToAngle extends CommandBase {
    double kMinimumAbsoluteOutput;
    double kTarget;
    double kCurrentPosition;
    double kIncrementMultiplier;
    /**
     * Creates a new MoveTurretAngle.
     */

    double angle;
    Turret turret;

    StepControl stepControl;

    public MoveTurretToAngle(double angle) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.turret = Robot.getRobotContainer().getTurret();

        addRequirements(turret);

        this.angle = angle;

        kMinimumAbsoluteOutput = RobotMap.TURRET_DEFAULT_MINIMUM_ABSOLUTE_OUTPUT;
        kTarget = - turret.getRotationsFromAngle(this.angle);
        kCurrentPosition =  turret.getTurretEncoder().getPosition();

        kIncrementMultiplier = RobotMap.TURRET_DEFAULT_kINCREMENT_MULTIPLIER;

        stepControl = new StepControl(kMinimumAbsoluteOutput, kTarget, kCurrentPosition, kIncrementMultiplier);

        stepControl.setRange(RobotMap.TURRET_DEFAULT_TARGET_RANGE);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        SmartDashboard.putNumber("turret - target", stepControl.getTarget());
        SmartDashboard.putNumber("turret - position", stepControl.getCurrentPosition());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double rotations = turret.getTurretEncoder().getPosition();

        double speed = stepControl.getOutputPosition(rotations);

        turret.setTurretRaw( speed);

        SmartDashboard.putNumber("turret - speed", speed);
        SmartDashboard.putNumber("turret - pos", rotations);
        SmartDashboard.putNumber("turret - minout", kMinimumAbsoluteOutput);
        SmartDashboard.putNumber("turret - target", kTarget);
        SmartDashboard.putNumber("turret - mult", kIncrementMultiplier);


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        turret.setTurretRaw(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return stepControl.isInRange();
    }
}
