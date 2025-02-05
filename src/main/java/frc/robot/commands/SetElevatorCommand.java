// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

// Raises Elevator
public class SetElevatorCommand extends Command {

  // Uses Elevator and Subsystems
  ElevatorSubsystem m_elevatorSubsystem;
  String m_level;

  // Constructor
  public SetElevatorCommand(ElevatorSubsystem elevatorSubsystem, String level) {
        
    // Definitions and setting parameters are equal to members!
    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);

    // Level
    m_level = level;
  }

  // Reset timer when the command starts executing
  public void initialize() {
  }
  
  // Actual command
  public void execute() {

    // Zero
    if (m_level == "zero") {
      m_elevatorSubsystem.setPosition(ElevatorConstants.k_zeroHeight);
    }

    // Raise to L2
    if(m_level == "L2") {
      m_elevatorSubsystem.setPosition(ElevatorConstants.k_coralL2Height);
    }

    // Raise to L3
    if(m_level == "L3") {
      m_elevatorSubsystem.setPosition(ElevatorConstants.k_coralL3Height);
    }

    // Raise to L4
    if(m_level == "L4") {
      m_elevatorSubsystem.setPosition(ElevatorConstants.k_coralL4Height);
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return true;
  }
}
