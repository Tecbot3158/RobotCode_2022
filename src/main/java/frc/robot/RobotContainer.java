// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.resources.Navx;
import frc.robot.subsystems.chassis.DriveTrain;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Feeder;
import frc.robot.subsystems.transport.Rollers;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.TecbotCamera;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private Intake intake;
    private DriveTrain driveTrain;
    private TecbotCamera tecbotCamera;

    private Turret turret;
    private Feeder feeder;
    private Rollers rollers;
    private Climber climber;

    private Navx gyroscope;

    private Shooter shooter;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        gyroscope = new Navx();
        tecbotCamera = new TecbotCamera();

        driveTrain = new DriveTrain();
        driveTrain.InitOdometry(gyroscope);

        intake = new Intake();

        turret = new Turret();

        feeder = new Feeder();

        rollers = new Rollers();

        shooter = new Shooter();

        climber = new Climber();


//        UsbCamera camera = CameraServer.startAutomaticCapture();
//
//        camera.setResolution(320, 240);

        // configureButtonBindings();
        // this NEEDS to be called from the RobotInit class after all subsystems are
        // initialized.

        // configureButtonBindings();
        // this NEEDS to be called from the RobotInit class after all subsystems are
        // initialized.

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    public void configureButtonBindings() {

        OI.getInstance().configureButtonBindings();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public Navx getNavx() {
        return gyroscope;
    }

    public Feeder getFeeder() {
        return feeder;
    }

    public Rollers getRollers() {
        return rollers;
    }

    public OI getOI() {

        return OI.getInstance();

    }

    public Intake getIntake() {
        return intake;
    }

    public Turret getTurret() {
        return turret;
    }

    public TecbotCamera getTecbotCamera() {
        return tecbotCamera;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Climber getClimber() {
        return climber;
    }

}
