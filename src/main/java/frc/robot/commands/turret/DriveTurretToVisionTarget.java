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
import frc.robot.subsystems.vision.TecbotCamera;

public class DriveTurretToVisionTarget extends CommandBase {

    private double kMinimumAbsoluteOutput;
    private double kCurrentPosition;
    private double kIncrementMultiplier;
    private double kTarget;

    TecbotCamera vision;
    Turret turret;

    StepControl control;

    double range;

    /**
     * Creates a new MoveTurretToTarget.
     */
    public DriveTurretToVisionTarget() {
        // Use addRequirements() here to declare subsystem dependencies.

        vision = Robot.getRobotContainer().getTecbotCamera();
        turret = Robot.getRobotContainer().getTurret();
        addRequirements(turret, vision);

        kMinimumAbsoluteOutput = RobotMap.TURRET_VISION_DEFAULT_MINIMUM_ABSOLUTE_OUTPUT;
        kTarget = vision.getYaw();
        // kCurrentPosition = turret.getTurretMotor().get();
        kIncrementMultiplier = RobotMap.TURRET_VISION_DEFAULT_kINCREMENT_MULTIPLIER;
        range = RobotMap.TURRET_VISION_DEFAULT_TARGET_RANGE;

        control = new StepControl(kMinimumAbsoluteOutput, kTarget, kCurrentPosition, kIncrementMultiplier);
        control.setTarget(range);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        vision.update();
        double yaw = vision.getYaw();

        kTarget = yaw;
        // kCurrentPosition = turret.getTurretMotor().get();

        // control.setCurrentPosition(kCurrentPosition);
        control.setTarget(kTarget);

        double speed = control.getOutputPosition(kCurrentPosition);

        turret.setTurretRaw(speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        turret.setTurretRaw(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean inRange = control.isInRange();
        SmartDashboard.putBoolean("Vision IN RANGE ", inRange);

        return control.isInRange();
    }
}
