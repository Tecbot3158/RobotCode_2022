// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.PositionStepControl;
import frc.robot.subsystems.turret.Turret;

public class DriveTurretToAngle extends CommandBase {
    double kMinimumAbsoluteOutput;
    double kTarget;
    double kCurrentPosition;
    double kStartControlLimit;


    double angle;
    Turret turret;

    boolean inRange = false;
    PositionStepControl stepControl;

    public DriveTurretToAngle(double angleDegrees) {
        // Use addRequirements() here to declare subsystem dependencies.

        this.turret = Robot.getRobotContainer().getTurret();

        addRequirements(turret);

        // convert from angles relative to robot
        // to real turret angles.
        // angle = 180 - angle;

        //if (inRange = turret.withinAngleRange(angle))
        //    this.angle = turret.getRealAngle(angle);

        this.angle = angleDegrees;

        kMinimumAbsoluteOutput = RobotMap.TURRET_DEFAULT_MINIMUM_ABSOLUTE_OUTPUT;
        kTarget = turret.getRotationsFromAngle(this.angle);
//        Robot.debug("angle: " +  this.angle);
//        Robot.debug("target: " +  this.kTarget);
        System.out.println("angle: " + this.angle);
        System.out.println("target: " + this.kTarget);
        kCurrentPosition = turret.getTurretEncoder().getPosition();

        kStartControlLimit = RobotMap.TURRET_DEFAULT_kCONTROL_START_LIMIT;

        stepControl = new PositionStepControl(kMinimumAbsoluteOutput, kTarget, kStartControlLimit);

        stepControl.setRange(RobotMap.TURRET_DEFAULT_TARGET_RANGE);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

//        SmartDashboard.putNumber("turret - target", stepControl.getTarget());
//        SmartDashboard.putNumber("turret - position", stepControl.getCurrentPosition());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

//        if (!inRange)
//            return;

        double rotations = turret.getTurretEncoder().getPosition();

        double speed = stepControl.getOutputPosition(rotations);

        turret.setTurretRaw(speed);

        SmartDashboard.putNumber("turret - speed", speed);
        SmartDashboard.putNumber("turret - pos", rotations);
        SmartDashboard.putNumber("turret - minout", kMinimumAbsoluteOutput);
        SmartDashboard.putNumber("turret - target", kTarget);
        SmartDashboard.putNumber("turret - mult", kStartControlLimit);


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        turret.setTurretRaw(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return stepControl.isInRange() || !inRange;
        return stepControl.isInRange();
    }
}
