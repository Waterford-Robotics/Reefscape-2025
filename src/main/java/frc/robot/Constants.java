// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.SwerveSubsystem;

public final class Constants {

  // Constants for Controller IDs
  public static final class OperatorConstants {
    public static final int k_driverController = 0;
  }

  // Constants for Kraken Drivetrain!
  public static final class SwerveConstants {
    // Must be max physically possible speed
    public static final double k_maxSpeed = edu.wpi.first.math.util.Units.feetToMeters(18.9);
  }

  // Constants for controller input!
  public static final class DriveConstants {

    // YAGSL Swerve Stuff (Don't touch)
    public static final double k_driveDeadBand = 0.1;
    public static final double k_driveSpeed = -1;
    public static final double k_turnRate = -1;

    // Driver Controls 
    public final static int k_rightbump = Button.kRightBumper.value; // Right Bump
    public final static int k_righttrig = Axis.kRightTrigger.value; // Right Trig
    public final static int k_lefttrig = Axis.kLeftTrigger.value; // Left Trig
    public final static int k_start = Button.kStart.value; // Start Button
    public final static int k_A = Button.kA.value; // A
    public final static int k_B = Button.kB.value; // A
    public final static int k_X = Button.kX.value; // A
    public final static int k_Y = Button.kY.value; // A
  }

  // Constants for Motor IDs
  public static final class MotorIDConstants {

    // Elevator
    public static final int k_elevatorKrakenLeftID = 21;
    public static final int k_elevatorKrakenRightID = 22;
  }

  // Constants for Sensor IDs
  public static final class SensorIDConstants {
    
    // REV Through Bore
    public static final int elevatorEncAPort = 0;
    public static final int elevatorEncBPort = 1;
  }

  // Constants for Elevator
  public static final class ElevatorConstants { // TODO: Change us!
    public static final Measure<Distance> k_coralL1Height = Units.Inches.of(10); 
    public static final Measure<Distance> k_coralL2Height = Units.Inches.of(20);
    public static final Measure<Distance> k_coralL3Height = Units.Inches.of(30);
    public static final Measure<Distance> k_coralL4Height = Units.Inches.of(40);
  }

  // Constants for Motors
  public static final class MotorConstants {
    public static final double k_rampRate = 0.05;
    public static final double k_closedMaxSpeed = 0.8;
    public static final int k_supplyCurrentLimit = 40;
  }

  // Constants for PID
  public static final class MotorPIDConstants {
    public static final double k_elevatorkP = 0.0;
    public static final double k_elevatorkI = 0.0;
    public static final double k_elevatorkD = 0.0;
    public static final double k_elevatorkS = 0.0;
    public static final double k_elevatorkV = 0.0;
    public static final double k_elevatorkG = 0.0;
  }

  // Constants for Autonomous
  public static final class AutoConstants {
    public static final PIDConstants k_translationPID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants k_anglePID = new PIDConstants(0.4, 0, 0.01);
    public static final ReplanningConfig k_replanningConfig = new ReplanningConfig();

    public static final HolonomicPathFollowerConfig k_autoConfig =  new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    AutoConstants.k_translationPID,
    // Translation PID constants
    AutoConstants.k_anglePID,
    // Rotation PID constants
    4.5,
    // Max module speed, in m/s
    SwerveSubsystem.swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
    // Drive base radius in meters. Distance from robot center to furthest module.
    new ReplanningConfig()
    // Default path replanning config. See the API for the options here
    );
  }

  // Limelight stuff yay
  public static final class LimelightConstants {

    // Tag Reject Distance
    public static final int rejectionDistance = 3;

    // Tag Reject Rotation Rate
    public static final int rejectionRotationRate = 720;
  }
}
