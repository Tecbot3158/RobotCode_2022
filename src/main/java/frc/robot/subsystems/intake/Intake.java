package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotSpeedController;

public class Intake extends SubsystemBase {

    DoubleSolenoid intakeSolenoids;
    TecbotSpeedController intakeMotor;

    IntakeMotorState intakeMotorState = IntakeMotorState.OFF;

    // constructor
    public Intake() {

        intakeSolenoids = RobotConfigurator.buildDoubleSolenoid(RobotMap.INTAKE_SOLENOID_PORTS);
        intakeMotor = new TecbotSpeedController(RobotMap.INTAKE_MOTOR_PORT, RobotMap.INTAKE_MOTOR_TYPE);
        intakeMotor.setInverted(RobotMap.INTAKE_MOTOR_IS_INVERTED);

        // intakeRetract();
        intakeMotorsOff();
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
        System.out.println(" intak reteract !!");
    }

    public void intakeExtend() {
        intakeSolenoids.set(RobotMap.INTAKE_POSITION_ACTIVE);
        System.out.println(" intak extend !!");
    }

    public void intakeMotorsAbsorb() {
        intakeMotor.set(RobotMap.INTAKE_ABSORB_SPEED);
    }

    public void intakeMotorsEject() {
        intakeMotor.set(RobotMap.INTAKE_EJECT_SPEED);
    }

    public void toggleIntakeMotorsEject() {
        intakeMotor.set(RobotMap.INTAKE_EJECT_SPEED);
    }

    public void intakeMotorsOff() {
        intakeMotor.set(RobotMap.INTAKE_OFF);
    }

    public void toggleIntakeMotorsAndPosition() {
        if (intakeSolenoids.get() == RobotMap.INTAKE_POSITION_RETRACTED) {
            intakeExtend();
            intakeMotorsAbsorb();
        } else {
            intakeRetract();
            intakeMotorsOff();
        }
    }

    public void toggleIntakePosition() {
        if (intakeSolenoids.get() == RobotMap.INTAKE_POSITION_RETRACTED)
            intakeExtend();
        else
            intakeRetract();
    }

    public void toggleIntakeMotors() {

    }

    public void setIntakeMotorsState(IntakeMotorState state) {
        intakeMotorState = state;

        switch (state) {
            case ABSORB:
                intakeMotorsAbsorb();
                break;
            case EJECT:
                intakeMotorsEject();
                break;
            default:
                intakeMotorsOff();
        }

    }

    public enum IntakeMotorState {
        ABSORB,
        EJECT,
        OFF

    }

    public IntakeMotorState getIntakeMotorState() {
        return intakeMotorState;
    }
}
