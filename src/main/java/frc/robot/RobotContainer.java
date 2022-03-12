// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.resources.Navx;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.chassis.DriveTrain;
import frc.robot.subsystems.intake.Intake;

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

    private Navx gyroscope;

    private Intake intake;

    private DriveTrain driveTrain;

    private OI oi;

<<<<<<< HEAD
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
=======
  private Turret turret;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
>>>>>>> 7b53d3922cbe5cf710c93eddee708e0afd8f0368

        gyroscope = new Navx();

        driveTrain = new DriveTrain();

        intake = new Intake();

<<<<<<< HEAD
        // configureButtonBindings();
        // this NEEDS to be called from the RobotInit class after all subsystems are
        // initialized.
=======
    turret = new Turret();

    // configureButtonBindings();
    // this NEEDS to be called from the RobotInit class after all subsystems are
    // initialized.
>>>>>>> 7b53d3922cbe5cf710c93eddee708e0afd8f0368

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        oi.configureButtonBindings();
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

    public Navx getOI() {
        return gyroscope;
    }

<<<<<<< HEAD
    public Intake getIntake() {
        return intake;
    }
=======
  public Intake getIntake() {
    return intake;
  }

  public Turret getTurret() {
    return turret;
  }

>>>>>>> 7b53d3922cbe5cf710c93eddee708e0afd8f0368
}
