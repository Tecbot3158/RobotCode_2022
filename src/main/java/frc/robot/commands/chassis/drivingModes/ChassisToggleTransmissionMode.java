/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.chassis.drivingModes;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.misc.PilotRumbleAllOff;
import frc.robot.subsystems.chassis.DriveTrain;

public class ChassisToggleTransmissionMode extends CommandBase {
    public ChassisToggleTransmissionMode() {

    }

    @Override
    public void initialize() {
        if (Robot.getRobotContainer().getDriveTrain().getTransmissionMode() == DriveTrain.TransmissionMode.speed) {
            Robot.getRobotContainer().getDriveTrain().setTransmissionState(DriveTrain.TransmissionMode.torque);
        }
        else {
            Robot.getRobotContainer().getDriveTrain().setTransmissionState(DriveTrain.TransmissionMode.speed);
            OI.getInstance().getPilot().setRumble(GenericHID.RumbleType.kLeftRumble, 1);
            OI.getInstance().getPilot().setRumble(GenericHID.RumbleType.kRightRumble, 1);
            new WaitCommand(0.3).andThen( new PilotRumbleAllOff()).schedule();

        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
