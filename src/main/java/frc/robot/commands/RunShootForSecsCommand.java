// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Raises Elevator
public class RunShootForSecsCommand extends Command {

  // Uses Elevator and Subsystems
  WristSubsystem m_wristSubsystem;
  double m_seconds;
  Timer m_timer = new Timer();

  // Constructor
  public RunShootForSecsCommand(WristSubsystem wristSubsystem, double seconds) {
        
    // Definitions and setting parameters are equal to members!
    m_wristSubsystem = wristSubsystem;
    addRequirements(wristSubsystem);

    // Level
    m_seconds = seconds;
  }

  // Reset timer when the command starts executing
  public void initialize() {
    m_timer.start();
    m_timer.reset();
  }
  
  // Actual command
  public void execute() {

    if(m_timer.get() < m_seconds) {
      m_wristSubsystem.shoot();
    }
  }

  // Stuff that happens when command is over
  public void end(boolean interrupted) {
    m_wristSubsystem.stopShooter();
  }

  // Checks if the command is done
  public boolean isFinished() {

    // Am I done?  Am I done? Am I finally done?
    return m_timer.get() > m_seconds;
  }
}
