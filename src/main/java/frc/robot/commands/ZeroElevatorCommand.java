// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

// Raises Elevator
public class ZeroElevatorCommand extends Command {

  // Uses Elevator and Subsystems
  ElevatorSubsystem m_elevatorSubsystem;
  boolean m_finished;

  // Constructor
  public ZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Reset timer when the command starts executing
  public void initialize() {
    m_finished = false;
  }
  
  // Actual command
  public void execute() {
    if(m_elevatorSubsystem.getCurrentPosition() < 5 && m_elevatorSubsystem.getCurrentVelocity() == 0) {
      m_finished = true;
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {

    // Neutral Motors and Reset Encoder Values
    m_elevatorSubsystem.setNeutral();
    m_elevatorSubsystem.resetSensorPosition(ElevatorConstants.k_zeroHeight);
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return m_finished;
  }
}
