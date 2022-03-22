// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.turret.Turret;

public class DriveTurretToAngleRelativeToRobotWithAxis extends CommandBase {
    double kMinimumAbsoluteOutput;
    double kTarget;
    double kCurrentPosition;
    double kIncrementMultiplier;
    /**
     * Creates a new MoveTurretAngle.
     */

    double angle;
    Turret turret;

    boolean withinAngleRange = false;
    StepControl stepControl;

    public DriveTurretToAngleRelativeToRobotWithAxis() {
        // Use addRequirements() here to declare subsystem dependencies.

        this.turret = Robot.getRobotContainer().getTurret();

        addRequirements(turret);

        // convert from angles relative to robot
        // to real turret angles.
        angle = 180 - angle;

        if (withinAngleRange = turret.withinAngleRange(angle))
            this.angle = turret.getRealAngle(angle);

        kMinimumAbsoluteOutput = RobotMap.TURRET_DEFAULT_MINIMUM_ABSOLUTE_OUTPUT;
        kTarget = turret.getRotationsFromAngle(this.angle);

        kCurrentPosition = turret.getTurretEncoder().getPosition();
        kIncrementMultiplier = RobotMap.TURRET_DEFAULT_kCONTROL_START_LIMIT;
        stepControl = new StepControl(kMinimumAbsoluteOutput, kTarget, kCurrentPosition, kIncrementMultiplier);

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

        double y = -OI.getInstance().getCopilot().getRightAxisY();
        double x = OI.getInstance().getCopilot().getRightAxisX();
        double angleDegrees = - Math.toDegrees(Math.atan2(y, x)) + 90;

        //    double target = OI.getInstance().getTurretInputAngle();
        kTarget = (turret.getRealAngle(angleDegrees)) ;

        SmartDashboard.putNumber("angle", angleDegrees);
        SmartDashboard.putNumber("angleKK", kTarget);
        kTarget = turret.getRotationsFromAngle(kTarget);

        double rotations = 0;
        double speed = 0;

        if (Math.abs(y) <= 0.3 && Math.abs(x) <= 0.3) {
            x = 0;
            y = 0;
            turret.setTurretRaw(0);
        } else {
            stepControl.setTarget(kTarget);
            rotations = turret.getTurretEncoder().getPosition();
            speed = stepControl.getOutputPosition(rotations);
            withinAngleRange = turret.withinAngleRange(angle);
            if (!withinAngleRange)
                return;

            turret.setTurretRaw(speed);
        }

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
        // return stepControl.isInRange() || !inRange;
        return false;
    }
}
