package frc.robot.commands.chassis.autonomous.splines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.resources.Math;
import frc.robot.resources.splines.CubicSpline;
import frc.robot.Robot;
import frc.robot.OI;

public class SplineMove extends CommandBase {

    CubicSpline spline;
    double maxSpeed;
    boolean speedReduction, vertical, inverted, reverse;

    /**
     * Creates a new autonomous command in which the robot will follow the given
     * trajectory.
     *
     * @param spline         The spline that defines the trajectory that this
     *                       command will follow.
     * @param maxSpeed       The maximum speed that will be given to the robot at
     *                       any given time.
     * @param speedReduction True if the robot needs to stop at the end of the
     *                       command. False is useful when concatenating paths.
     * @param vertical       The default axis settings are: x-axis is paralell to
     *                       the guardrails, y-axis is paralel to the driver
     *                       stations. True if these settings need to be inverted.
     * @param inverted       By default, the trajectory is folllowed from left to
     *                       right. True if this setting needs to be inverted.
     * @param reverse        By default, the robot will follow the trajectory going
     *                       forward. True if the robot needs to follow the path
     *                       backwards.
     */
    public SplineMove(CubicSpline spline, double maxSpeed, boolean speedReduction, boolean vertical, boolean inverted,
            boolean reverse) {

        addRequirements(Robot.getRobotContainer().getDriveTrain());

        this.spline = spline;
        this.maxSpeed = maxSpeed;
        this.speedReduction = speedReduction;
        this.vertical = vertical;
        this.inverted = inverted;
        this.reverse = reverse;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

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
