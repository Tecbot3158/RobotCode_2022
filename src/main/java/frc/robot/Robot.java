// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.chassis.DefaultDrive;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallsAuto;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallsPath;
import frc.robot.commands.chassis.autonomous.fullAutos.ThreeBallsAuto;
import frc.robot.commands.chassis.autonomous.fullAutos.FiveBallNoSpline.FiveBallsNoSpline;
import frc.robot.commands.chassis.autonomous.fullAutos.TwoBalls.TwoBallsAuto;
import frc.robot.commands.chassis.autonomous.splines.Paths;
import frc.robot.commands.chassis.autonomous.splines.SplineMove;
import frc.robot.commands.chassis.autonomous.stepControl.SpeedReductionTurn;
import frc.robot.commands.chassis.drivingModes.ChassisSetSpeed;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotPWMLEDStrip;
import frc.robot.subsystems.chassis.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private static RobotContainer robotContainer;

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        robotContainer.configureButtonBindings();

        new ChassisSetSpeed().schedule();

        robotContainer.getDriveTrain().setDefaultCommand(new DefaultDrive());
        // DoubleSolenoid a = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

        robotContainer.getNavx().reset();

        Paths paths = new Paths();
        // FiveBallsAuto fiveBalls = new
        // FiveBallsAuto(paths.getFiveBallAutoPathSpline());
        FiveBallsNoSpline fiveBalls = new FiveBallsNoSpline();

        m_chooser.addOption("None", null);
        m_chooser.addOption("2 Balls", new TwoBallsAuto());
        m_chooser.addOption("3 Balls", new ThreeBallsAuto());
        m_chooser.setDefaultOption("fiveBalls", fiveBalls);
        SmartDashboard.putData("Drivetrain", Robot.getRobotContainer().getDriveTrain());

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        m_autonomousCommand = m_chooser.getSelected();
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Gyro", getRobotContainer().getNavx().getYaw());

        SmartDashboard.putNumber("turret encoder:",
                Robot.getRobotContainer().getTurret().getTurretEncoder().getPosition());
        SmartDashboard.putNumber("turret RPMs:",
                Robot.getRobotContainer().getTurret().getTurretEncoder().getVelocity());

        SmartDashboard.putData("Autonomous", m_chooser);
        // System.out.println(getRobotContainer().getIntake().getIntakeSolenoids().get());
        // OI.getInstance().getTurretInputAngle();

    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

        SmartDashboard.putData("Autonomous", m_chooser);
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        getRobotContainer().getDriveTrain().setTransmissionState(DriveTrain.TransmissionMode.speed);
        // always put chassi for speed !

        m_autonomousCommand = m_chooser.getSelected();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        double ledInput = OI.getInstance().getDefaultDriveInputY();
        // ;

        // this works:
        // robotContainer.getTecbotPWMLEDStrip().setRainbowCycle();

        // this doesn't work!
        // robotContainer.getTecbotPWMLEDStrip().setFireFromInputInRange(1, 0.1, 0, 1,
        // 60, 1, false);

        // this works!
        // robotContainer.getTecbotPWMLEDStrip().setSolidHSV(0, 0, 0);

        // this probably works
        robotContainer.getTecbotPWMLEDStrip().setFireWithVariableIntensity(
                233, Math.abs(ledInput), 0, 1, TecbotConstants.LED_STRIP_LENGTH, 0, false);

        // robotContainer.getTecbotPWMLEDStrip().setFireFromJoystick(129, x, y,
        // TecbotConstants.LED_STRIP_LENGTH, 1,
        // true);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    public static RobotContainer getRobotContainer() {
        return robotContainer;
    }

    public static void debug(String value) {
        if (TecbotConstants.DEBUG_ENABLED)
            System.out.println(value);

    }

    public static void debugSmartDashboard(String key, double value) {
        if (TecbotConstants.DEBUG_ENABLED)
            SmartDashboard.putNumber(key, value);
    }
}
