package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.StepControl;
import frc.robot.resources.TecbotConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.resources.VisionValueNormalizer;

public class ShooterGoToVariableVisionTarget extends CommandBase {

    StepControl stepControl;

    Shooter shooter;

    double target, currentPos;

    double increment;

    double minimumOutput;

    public ShooterGoToVariableVisionTarget() {

        shooter = Robot.getRobotContainer().getShooter();
        Subsystem requirements[] = {shooter};
        addRequirements(requirements);

        shooter.getShooterEncoder().setPosition(0);

    }

    @Override
    public void initialize() {

        currentPos = shooter.getShooterEncoder().getVelocity();

        increment = RobotMap.SHOOTER_CONTROL_kINCREMENT_MULTIPLIER;
        minimumOutput = RobotMap.SHOOTER_CONTROL_MINIMUM_OUTPUT;

        stepControl = new StepControl(minimumOutput, target, currentPos, increment);
        stepControl.setRange(RobotMap.SHOOTER_DEFAULT_RANGE);

        Robot.debugSmartDashboard("SHOOT-MIN_OUT", minimumOutput);
        Robot.debugSmartDashboard("SHOOT-TARGET", target);
        Robot.debugSmartDashboard("SHOOT-INCREM", increment);

        Robot.debug("init SHOOTER");

    }

    @Override
    public void execute() {

        double normalizeDistance = VisionValueNormalizer.getInstance().getNormalizedDistance();
        target = RobotMap.TURRET_VISION_MINIMUM_RPMs +
                normalizeDistance * RobotMap.TURRET_VISION_MAXIMUM_VARIABLE_RPMs;

        stepControl.setTarget(target);

        double velocity = shooter.getShooterEncoder().getVelocity();
        double output = stepControl.getOutputVelocity(velocity);

        Robot.debugSmartDashboard("shooter encoder. ", shooter.getShooterEncoder().getPosition());
        Robot.debugSmartDashboard("shooter output ", output);
        Robot.debugSmartDashboard("shooter RPMs", velocity);
        Robot.debugSmartDashboard("shooter target", target);

        shooter.setShooterMotorsRaw(output);

        if (stepControl.isInRange()) {
            OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble,
                    TecbotConstants.COPILOT_DEFAULT_VIBRATION);
            OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble,
                    TecbotConstants.COPILOT_DEFAULT_VIBRATION);
        } else {
            OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble, 0);

        }

    }

    @Override
    public void end(boolean interrupted) {
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}