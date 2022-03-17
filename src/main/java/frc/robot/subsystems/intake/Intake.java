package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotSpeedController;

public class Intake extends SubsystemBase {

    DoubleSolenoid intakeSolenoids;
    TecbotSpeedController intakeMotor;

    // constructor
    public Intake() {

        intakeSolenoids = RobotConfigurator.buildDoubleSolenoid(RobotMap.INTAKE_SOLENOID_PORTS);
        intakeMotor = new TecbotSpeedController(RobotMap.INTAKE_MOTOR_PORT, RobotMap.INTAKE_MOTOR_TYPE);
    }

    // getters
    public TecbotSpeedController getIntakeMotor() {
        return intakeMotor;
    }

    public DoubleSolenoid getIntakeSolenoids() {
        return intakeSolenoids;
    }

    // actions
    public void intakeRetract() {
        intakeSolenoids.set(RobotMap.INTAKE_POSITION_RETRACTED);
    }

    public void intakeExtend() {
        intakeSolenoids.set(RobotMap.INTAKE_POSITION_ACTIVE);
    }

    public void intakeMotorsAbsorb() {
        intakeMotor.set(RobotMap.INTAKE_ABSORB_SPEED);
    }

    public void intakeMotorsEject() {
        intakeMotor.set(RobotMap.INTAKE_EJECT_SPEED);
    }

    public void intakeMotorsOff() {
        intakeMotor.set(RobotMap.INTAKE_OFF);
    }

    public void toggleIntake() {
        if (intakeSolenoids.get() == RobotMap.INTAKE_POSITION_RETRACTED) {
            intakeExtend();
            intakeMotorsAbsorb();
        } else {
            intakeRetract();
            intakeMotorsOff();
        }
    }
}
