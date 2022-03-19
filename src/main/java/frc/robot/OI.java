package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import frc.robot.commands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.chassis.drivingModes.ToggleMecanum;
import frc.robot.commands.feeder.FeederRaw;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.intake.basic.IntakeExtend;
import frc.robot.commands.intake.basic.IntakeRetract;
import frc.robot.commands.intake.basic.IntakeToggle;
import frc.robot.commands.oitests.OITestTriggers;
import frc.robot.commands.rollers.RollersMove;
import frc.robot.commands.rollers.SetRollersRaw;
import frc.robot.commands.shooter.ShooterGoRaw;
import frc.robot.commands.shooter.ShooterGoToTarget;
import frc.robot.commands.turret.DriveTurretManually;
import frc.robot.commands.turret.DriveTurretRaw;
import frc.robot.commands.turret.MoveTurretToCenter;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotController;

public class OI {

    public static OI instance;

    private TecbotController pilot, copilot;

    private OI() {

        pilot = new TecbotController(0, TecbotController.TypeOfController.XBOX);
        copilot = new TecbotController(1, TecbotController.TypeOfController.PS4);

    }

    /**
     * must always be called after ALL subsystems are
     * created.
     */
    public void configureButtonBindings() {

        pilot = new TecbotController(0, TecbotConstants.CONTROLLER_TYPE_PILOT);
        copilot = new TecbotController(1, TecbotConstants.CONTROLLER_TYPE_COPILOT);


        // pilot.whileHeld(TecbotController.ButtonType.POV_0, new ShooterGoRaw(0.5));
        //pilot.whileHeld(TecbotController.ButtonType.POV_180, new ShooterGoRaw(0.5));

        pilot.whenPressed(TecbotController.ButtonType.X, new ToggleMecanum());
        pilot.whenPressed(TecbotController.ButtonType.A, new ChassisSetDefaultDrive());


        //pilot.whenPressed(TecbotController.ButtonType.RB, new FeederSetToSpeed());

//        pilot.whenPressed(TecbotController.ButtonType.START, new DriveTurretRaw( 0.1 ));
//        pilot.whenPressed(TecbotController.ButtonType.BACK, new DriveTurretRaw( -0.1 ));

        //pilot.whenPressed(TecbotController.ButtonType.START, new IntakeExtend());
        //pilot.whenPressed(TecbotController.ButtonType.BACK, new IntakeRetract());

        pilot.whenPressed(TecbotController.ButtonType.POV_UP, new IntakeToggle());
        pilot.whenPressed(TecbotController.ButtonType.START, new SetRollersRaw(0.8));


        // 0.6 disparador

        pilot.whenPressed(TecbotController.ButtonType.BACK, new FeederRaw(1));
        pilot.whileHeld(TecbotController.ButtonType.RB, new ShooterGoToTarget());

        // pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new DriveTurretManually());

        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new MoveTurretToCenter() );


        /*
        un mismo boton
         */

    }

    // singleton
    public static OI getInstance() {
        if (instance == null)
            instance = new OI();

        return instance;
    }

    public double getRollersInput() {
        return pilot.getRightAxisX();
    }

    public TecbotController getPilot() {

        return pilot;
    }

    public TecbotController getCopilot() {

        return copilot;

    }

}
