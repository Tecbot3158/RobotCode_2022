package frc.robot.resources;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class TecbotSensors {

    private Navx tecbotGyro;

    private TecbotEncoder leftChassisEncoder, rightChassisEncoder, middleChassisEncoder,
            sharedMotorsRightEncoder, sharedMotorsLeftEncoder;


    //CLIMBER stuff
    private DigitalInput climberLeftLimitSwitch;
    private DigitalInput climberRightLimitSwitch;



    public TecbotSensors() {

    }

    /**
     * Initializes all sensors.
     */
    public void initializeAllSensors() {

        tecbotGyro = new Navx();
        /*sharedMotorsRightEncoder = RobotConfigurator.buildEncoder(Robot.getRobotContainer().getSharedMotors().getMotorWithEncoderRight(),
                RobotMap.SHARED_MOTORS_RIGHT_ENCODER_PORTS[0],
                RobotMap.SHARED_MOTORS_RIGHT_ENCODER_PORTS[1]);*/
        /*sharedMotorsLeftEncoder = RobotConfigurator.buildEncoder(Robot.getRobotContainer().getSharedMotors().getMotorWithEncoderLeft(),
                RobotMap.SHARED_MOTORS_LEFT_ENCODER_PORTS[0],
                RobotMap.SHARED_MOTORS_LEFT_ENCODER_PORTS[1]);
*/
        leftChassisEncoder = RobotConfigurator.buildEncoder
                (Robot.getRobotContainer().getDriveTrain().getSpecificMotor(RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_MOTOR_WITH_ENCODER),
                        RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_PORTS[0], RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_PORTS[1]);
        rightChassisEncoder = RobotConfigurator.buildEncoder
                (Robot.getRobotContainer().getDriveTrain().getSpecificMotor(RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_MOTOR_WITH_ENCODER),
                        RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_PORTS[0], RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_PORTS[1]);
        middleChassisEncoder = RobotConfigurator.buildEncoder
                (Robot.getRobotContainer().getDriveTrain().getSpecificMotor(RobotMap.DRIVE_TRAIN_MIDDLE_CHASSIS_MOTOR_WITH_ENCODER)
                        , RobotMap.DRIVE_TRAIN_MIDDLE_WHEEL_ENCODER_PORTS[0], RobotMap.DRIVE_TRAIN_MIDDLE_WHEEL_ENCODER_PORTS[1]);
        if (RobotMap.DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_IS_INVERTED && leftChassisEncoder != null)
            leftChassisEncoder.setInverted(true);
        if (RobotMap.DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_IS_INVERTED && rightChassisEncoder != null)
            rightChassisEncoder.setInverted(true);
        if (RobotMap.DRIVE_TRAIN_MIDDLE_CHASSIS_ENCODER_IS_INVERTED && middleChassisEncoder != null)
            middleChassisEncoder.setInverted(true);


        climberLeftLimitSwitch = new DigitalInput(RobotMap.CLIMBER_LEFT_LIMIT_SWITCH_PORT);
        climberRightLimitSwitch = new DigitalInput(RobotMap.CLIMBER_RIGHT_LIMIT_SWITCH_PORT);

    }

    /**
     * Must be called to update tecbotGyro data and InfraRed sensors for
     * Power cell counting.
     */
    public void sensorsPeriodic() {
        tecbotGyro.run();

        //SmartDashboard.putBoolean("IR shooter", infraredShooterSensor.get());
        //infraredFrontIntakeSensor.debug();
        //infraredRearIntakeSensor.debug("RI");


    }

    /**
     * @return {@link Navx} object of tecbotGyro
     */
    public Navx getTecbotGyro() {
        return tecbotGyro;
    }

    /**
     * @return angle between -180 and 180.
     */
    public double getYaw() {
        return tecbotGyro.getYaw();
    }

    /**
     * @return Raw value from selected encoder
     */
    public double getEncoderRaw(SubsystemType subsystem) {
        switch (subsystem) {
            case RIGHT_CHASSIS:
                return rightChassisEncoder.getSparkRaw();
            case LEFT_CHASSIS:
                return leftChassisEncoder.getSparkRaw();
            case MIDDLE_CHASSIS:
                return middleChassisEncoder.getSparkRaw();
            case SHOOTER:
                return RobotMap.SHOOTER_ENCODER_IN_RIGHT_MOTOR ?
                        //sharedMotorsRightEncoder.getRaw() :
                        0:
                        0;
                        //
            // sharedMotorsLeftEncoder.getRaw();
            default:
                return 0;
        }
    }

    public TecbotEncoder getEncoder(SubsystemType subsystem) {
        switch (subsystem) {
            case RIGHT_CHASSIS:
                return rightChassisEncoder;
            case LEFT_CHASSIS:
                return leftChassisEncoder;
            case MIDDLE_CHASSIS:
                return middleChassisEncoder;
            case SHOOTER:
                return RobotMap.SHOOTER_ENCODER_IN_RIGHT_MOTOR ?
                        sharedMotorsRightEncoder :
                        sharedMotorsLeftEncoder;
            default:
                return null;
        }


    }

    /**
     * subsystem type
     */
    public enum SubsystemType {
        MIDDLE_CHASSIS, RIGHT_CHASSIS, LEFT_CHASSIS, SHOOTER
    }



}
