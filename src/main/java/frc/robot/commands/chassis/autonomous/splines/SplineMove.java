package frc.robot.commands.chassis.autonomous.splines;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.resources.Math;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotSpeedController;
import frc.robot.resources.splines.PieceWiseSpline;
import frc.robot.Robot;

public class SplineMove extends CommandBase {

    PieceWiseSpline spline;
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
    public SplineMove(PieceWiseSpline spline, double maxSpeed, boolean speedReduction, boolean vertical,
            boolean inverted,
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

        System.out.println("Endend spline move");
        Robot.getRobotContainer().getDriveTrain().stop();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        DifferentialDriveOdometry odometry = Robot.getRobotContainer().getDriveTrain().getOdometry();

        double error = 0;

        double currentAngle = 0;

        if (vertical) {

            error = odometry.getPoseMeters().getX() - spline.f(odometry.getPoseMeters().getY());
            currentAngle = spline.angle(odometry.getPoseMeters().getY());

        } else {

            error = odometry.getPoseMeters().getY() - spline.f(odometry.getPoseMeters().getX());
            currentAngle = spline.angle(odometry.getPoseMeters().getX());

        }

        if (reverse) {
            currentAngle += error * TecbotConstants.SPLINE_DRIVE_ERROR_CORRECTION;
            maxSpeed = -Math.abs(maxSpeed);
        } else {
            currentAngle += error * TecbotConstants.SPLINE_DRIVE_ERROR_CORRECTION;

        }

        if (inverted || reverse) {
            currentAngle = Math.getOppositeAngle(currentAngle);
        }

        double finalXPosition = spline.getFinalXPosition();

        if (inverted) {

            finalXPosition = spline.getInitialXPosition();

        }
        double finalYPosition = spline.f(finalXPosition);

        double distanceToTarget = Math.distance(odometry.getPoseMeters().getX(), finalXPosition,
                odometry.getPoseMeters().getY(), finalYPosition);

        if (vertical) {

            distanceToTarget = Math.distance(odometry.getPoseMeters().getY(), finalXPosition,
                    odometry.getPoseMeters().getX(), finalYPosition);

        }

        double power = maxSpeed;

        if (speedReduction) {

            double speedReductionCorrection = Math
                    .clamp(distanceToTarget / TecbotConstants.SPLINE_SPEED_REDUCTION_MAX_DISTANCE, 0, 1);

            power = speedReductionCorrection * maxSpeed;
        }

        SmartDashboard.putNumber("Error", error);

        Robot.getRobotContainer().getDriveTrain().angleCorrectionDrive(power, currentAngle);

        return distanceToTarget < TecbotConstants.CHASSIS_SPLINE_ARRIVE_OFFSET;
    }

}
