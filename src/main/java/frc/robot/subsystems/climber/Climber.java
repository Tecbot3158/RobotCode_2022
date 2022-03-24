/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotMotorList;

public class Climber extends SubsystemBase {

    private final DoubleSolenoid pistonHanger;
    private final TecbotMotorList ropeController;

    private RelativeEncoder ropeEncoder;

    public Climber() {

        ropeController = RobotConfigurator.buildMotorList(RobotMap.CLIMBER_MOTOR_PORTS,
                RobotMap.CLIMBER_INVERTED_MOTORS, RobotMap.CLIMBER_MOTOR_TYPES);
        pistonHanger = RobotConfigurator.buildDoubleSolenoid(RobotMap.CLIMBER_SOLENOID_PORTS);

        CANSparkMax leaderMotor = ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_MASTER_PORT).getCANSparkMax();
        leaderMotor.setInverted(RobotMap.CLIMBER_MOTOR_MASTER_IS_INVERTED);

        CANSparkMax slaveMotor = ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_SLAVE_PORT).getCANSparkMax();

        slaveMotor.follow(leaderMotor, true);
        // MUST ALWATS be true when following another motor,
        // EVEN if the other motor is inverted.!!

        // santi says break !
        leaderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        slaveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        ropeEncoder = ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_MASTER_PORT).getCANSparkMax().getEncoder();

        ropeEncoder.setPosition(0);

        // TODO disable dragon fly in command for climber! Always!
        // TODO Dragon fly cannot be extended!!

    }

    public void setPistonExtend() {
        pistonHanger.set(RobotMap.CLIMBER_SOLENOID_EXTENDED_POSITION);
    }

    public DoubleSolenoid.Value getPistonState() {
        return pistonHanger.get();
    }

    public void setPistonRetract() {
        pistonHanger.set(RobotMap.CLIMBER_SOLENOID_RETRACTED_POSITION);
    }

    public void setPistonRaw(DoubleSolenoid.Value state) {
        pistonHanger.set(state);
    }

    public void setRopeControllerRaw(double speed) {

        double clampedSpeed = Math.clamp(speed, RobotMap.CLIMBER_RAW_MINIMUM_SPEED, RobotMap.CLIMBER_RAW_MAXIMUM_SPEED);
        double realSpeed = clampedSpeed;
        Robot.debugSmartDashboard("CLIMB - enc", ropeEncoder.getPosition());
        Robot.debugSmartDashboard("CLIMB - SPEED", realSpeed);

        boolean freeMode = true;
        if (!freeMode) {

            if (getPistonState() == RobotMap.CLIMBER_SOLENOID_RETRACTED_POSITION) {
                if (!withinRangePistonRetracted() && !withinRangeNegativePistonExtended())
                    realSpeed = Math.clamp(clampedSpeed, 0, RobotMap.CLIMBER_RAW_MAXIMUM_SPEED);
                else if (!withinRangeNegativePistonExtended() &&
                        !withinRangePositivePistonExtended())
                    realSpeed = Math.clamp(clampedSpeed, RobotMap.CLIMBER_RAW_MINIMUM_SPEED, 0);

                // if we are within the minimum and maximum range
                // do absolutely nothing and leave variable as is.
            } else {
                // means pistons are extended !
                if (!withinRangePistonExtended() && !withinRangeNegativePistonExtended())
                    realSpeed = Math.clamp(clampedSpeed, 0, RobotMap.CLIMBER_RAW_MAXIMUM_SPEED);
                if (!withinRangePistonExtended() && !withinRangePositivePistonExtended())
                    realSpeed = Math.clamp(clampedSpeed, RobotMap.CLIMBER_RAW_MINIMUM_SPEED, 0);
            }
            ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_MASTER_PORT).set(
                    realSpeed);
        } else {

            ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_MASTER_PORT).set(realSpeed);
        }

    }

    public boolean withinRangeNegativePistonExtended() {
        return (ropeEncoder.getPosition() > RobotMap.CLIMBER_ENCODER_MINIMUM_VALUE_PISTON_EXTENDED);
    }

    public boolean withinRangePositivePistonExtended() {
        return (ropeEncoder.getPosition() < RobotMap.CLIMBER_ENCODER_MAXIMUM_VALUE_PISTON_EXTENDED);
    }

    public boolean withinRangePistonExtended() {
        return (ropeEncoder.getPosition() > RobotMap.CLIMBER_ENCODER_MINIMUM_VALUE_PISTON_EXTENDED) &&
                (ropeEncoder.getPosition() < RobotMap.CLIMBER_ENCODER_MAXIMUM_VALUE_PISTON_EXTENDED);
    }

    public boolean withinRangePistonRetracted() {
        return (ropeEncoder.getPosition() > RobotMap.CLIMBER_ENCODER_MINIMUM_VALUE_PISTON_RETRACTED) &&
                (ropeEncoder.getPosition() < RobotMap.CLIMBER_ENCODER_MAXIMUM_VALUE_PISTON_RETRACTED);
    }

    public boolean withinRangeNegativePistonRetracted() {
        return (ropeEncoder.getPosition() > RobotMap.CLIMBER_ENCODER_MINIMUM_VALUE_PISTON_RETRACTED);
    }

    public boolean withinRangePositivePistonRetracted() {
        return (ropeEncoder.getPosition() < RobotMap.CLIMBER_ENCODER_MAXIMUM_VALUE_PISTON_RETRACTED);
    }

    public TecbotMotorList getRopeControllerList() {
        return ropeController;
    }

    /**
     * Returns the master CAN Spark Max {@link RelativeEncoder}
     *
     */
    public RelativeEncoder getRopeControllerEncoder() {
        return ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_MASTER_PORT).getCANSparkMax().getEncoder();
    }

}