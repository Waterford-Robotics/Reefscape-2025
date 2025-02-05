// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.SetElevatorCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.commands.RunIntakeForSecsCommand;
import frc.robot.commands.RunShootForSecsCommand;
import frc.robot.commands.AimNRangeReefLeftCommand;
import frc.robot.commands.AimNRangeReefRightCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();

  // Create New Choosing Option in SmartDashboard for Autos
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);

  // Command Chain for Scoring on L2
  SequentialCommandGroup scoreL2Command = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new SetElevatorCommand(m_elevatorSubsystem, "L2"),
      new RunIntakeForSecsCommand(m_wristSubsystem, 1.5)
    ),
    new RunShootForSecsCommand(m_wristSubsystem, 1),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Scoring on L3
  SequentialCommandGroup scoreL3Command = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new SetElevatorCommand(m_elevatorSubsystem, "L3"),
      new RunIntakeForSecsCommand(m_wristSubsystem, 1.5)
    ),
    new RunShootForSecsCommand(m_wristSubsystem, 1),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  // Command Chain for Scoring on L4
  SequentialCommandGroup scoreL4Command = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new SetElevatorCommand(m_elevatorSubsystem, "L4"),
      new RunIntakeForSecsCommand(m_wristSubsystem, 1.5)
    ),
    new RunShootForSecsCommand(m_wristSubsystem, 1),
    new SetElevatorCommand(m_elevatorSubsystem, "zero"),
    new ZeroElevatorCommand(m_elevatorSubsystem)
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // Makes the drive command the default command (good!)
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

    // Named Command Configuration
    // NamedCommands.registerCommand("ExampleCommand", new ExampleCommand(m_exampleSubsystem);

    // Autos
    m_chooser.addOption("Test Auto", m_swerveSubsystem.getAutonomousCommand("Test Auto"));
    m_chooser.addOption("Example Auto RED", m_swerveSubsystem.getAutonomousCommand("Example Auto RED"));
    m_chooser.addOption("Example Auto BLUE", m_swerveSubsystem.getAutonomousCommand("Example Auto BLUE"));

    // Puts a chooser on the SmartDashboard!
    SmartDashboard.putData("AutoMode", m_chooser);
  }

  // Trigger & Button Bindings!
  private void configureBindings() {

    // Lower Elevator Manually - "A" Button
    new JoystickButton(m_driverController.getHID(), DriveConstants.k_A)
      .onTrue(
        new SequentialCommandGroup(
          new InstantCommand(() -> m_elevatorSubsystem.setPosition(ElevatorConstants.k_zeroHeight), m_elevatorSubsystem),
          new InstantCommand(() -> m_elevatorSubsystem.setNeutral(), m_elevatorSubsystem)
        )
      );
    
    // Raise Elevator to L2 Manually - "B" Button
    new JoystickButton(m_driverController.getHID(), DriveConstants.k_B)
      .onTrue(
        scoreL2Command
      );

    // Raise Elevator to L3 Manually - "X" Button
    new JoystickButton(m_driverController.getHID(), DriveConstants.k_X)
      .onTrue(
        scoreL3Command
      );

    // Score Coral on L3 and Zero Elevator - "Y" Button
    new JoystickButton(m_driverController.getHID(), DriveConstants.k_Y)
      .onTrue(
        scoreL4Command
      );

    /*
    new JoystickButton(m_driverController.getHID(), DriveConstants.k_rightbump)
      .onTrue(
        new InstantCommand(() -> m_wristSubsystem.setPosition(WristConstants.k_wrist1Height))
      );

    new JoystickButton(m_driverController.getHID(), DriveConstants.k_leftbump)
      .onTrue(
        new InstantCommand(() -> m_wristSubsystem.setPosition(WristConstants.k_wrist2Height))
      );
    */

    // Intake Manually - Right Trig
    new Trigger(() -> m_driverController.getRawAxis(DriveConstants.k_righttrig) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_wristSubsystem.intake(), m_wristSubsystem)
      )
      .whileFalse(
        new InstantCommand(() -> m_wristSubsystem.stopShooter(), m_wristSubsystem)
      );

    // Shoot Manually - Left Trig
    new Trigger(() -> m_driverController.getRawAxis(DriveConstants.k_lefttrig) > 0.05)
      .whileTrue(
        new InstantCommand(() -> m_wristSubsystem.shoot(), m_wristSubsystem)
      )
      .whileFalse(
        new InstantCommand(() -> m_wristSubsystem.stopShooter(), m_wristSubsystem)
      );

    // Example Path yay - "start" Button
    new JoystickButton(m_driverController.getHID(), DriveConstants.k_start)
      .onTrue(
        new InstantCommand(() -> m_swerveSubsystem.followPathAutobuilderCommand("Example Path RED"), m_swerveSubsystem)
      );

    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadRight)
    .onTrue(
      new AimNRangeReefRightCommand(m_swerveSubsystem)
    );

    new POVButton(m_driverController.getHID(), ControllerConstants.k_dpadLeft)
    .onTrue(
      new AimNRangeReefLeftCommand(m_swerveSubsystem)
    );
  }
  
  // Commands!
  // Command that takes Xbox Controller Inputs and allows robot to drive
  // NOTE: getLeftY and getLeftX are opposite for a reason!!! It is correct!!
  public Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.k_driveSpeed, DriveConstants.k_driveDeadBand),
      () -> m_driverController.getRightX() * DriveConstants.k_turnRate);

  // Use this to pass the autonomous command to Robot.java
  // Returns the command to run in autonomous
  public Command getAutonomousCommand() {

    // The selected auto on SmartDashboard will be run in autonomous
    return m_chooser.getSelected(); 
  }
}
