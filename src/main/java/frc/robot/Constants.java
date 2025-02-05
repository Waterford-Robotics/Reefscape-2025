// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.SwerveSubsystem;

public final class Constants {

  public static final class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.10;

    public final static int k_start = Button.kStart.value; // Start Button
    public static final int k_A = Button.kA.value; // A
    public static final int k_B = Button.kB.value; // B
    public static final int k_X = Button.kX.value; // X
    public static final int k_Y = Button.kY.value; // Y
    public static final int k_dpadRight = 90; // D-Pad Right
    public static final int k_dpadLeft = 270; // D-Pad Left
}


  // Constants for Kraken Drivetrain!
  public static final class SwerveConstants {
    // Must be max physically possible speed
    public static final double k_maxSpeed = edu.wpi.first.math.util.Units.feetToMeters(18.9);
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second
  }

  // Constants for controller input!
  public static final class DriveConstants {

    // YAGSL Swerve Stuff (Don't touch)
    public static final double k_driveDeadBand = 0.1;
    public static final double k_driveSpeed = -1;
    public static final double k_turnRate = -1;

    // Driver Controls 
    public final static int k_rightbump = Button.kRightBumper.value; // Right Bump
    public final static int k_leftbump = Button.kLeftBumper.value; // Right Bump
    public final static int k_righttrig = Axis.kRightTrigger.value; // Right Trig
    public final static int k_lefttrig = Axis.kLeftTrigger.value; // Left Trig
    public final static int k_start = Button.kStart.value; // Start Button
    public final static int k_A = Button.kA.value; // A
    public final static int k_B = Button.kB.value; // B
    public final static int k_X = Button.kX.value; // X
    public final static int k_Y = Button.kY.value; // Y
  }

  // Constants for Motor IDs
  public static final class MotorIDConstants {

    // Elevator
    public static final int k_elevatorKrakenLeftID = 21;
    public static final int k_elevatorKrakenRightID = 22;

    // Wrist
    public static final int k_shooterKrakenID = 31;
    public static final int k_wristKrakenID = 32;
  }

  // Constants for Sensor IDs
  public static final class SensorIDConstants {
    
    // REV Through Bore
    public static final int elevatorEncAPort = 0;
    public static final int elevatorEncBPort = 1;
  }

  // Constants for Elevator
  public static final class ElevatorConstants {
    public static final Measure<Distance> k_zeroHeight = Units.Inches.of(0);
    public static final Measure<Distance> k_coralL1Height = Units.Inches.of(0); 
    public static final Measure<Distance> k_coralL2Height = Units.Inches.of(6);
    public static final Measure<Distance> k_coralL3Height = Units.Inches.of(22);
    public static final Measure<Distance> k_coralL4Height = Units.Inches.of(48.8);
  }

  // Constants for Wrist
  public static final class WristConstants { // TODO: Change us!
    public static final Measure<Angle> k_wrist1Height = Units.Rotations.of(0.96); 
    public static final Measure<Angle> k_wrist2Height = Units.Rotations.of(0.96);
    public static final Measure<Angle> k_coralIntakeAngle = Units.Rotations.of(1.3);
  }

  // Constants for Motors
  public static final class MotorConstants {
    public static final double k_elevatorRampRate = 0.25;
    public static final double k_elevatorClosedMaxSpeed = 0.8;
    public static final int k_elevatorSupplyCurrentLimit = 40;

    public static final double k_shooterRampRate = 0.05;
    public static final double k_shooterClosedMaxSpeed = 0.1;
    public static final int k_supplyCurrentLimit = 40;
  }

  // Constants for PID
  public static final class MotorPIDConstants {
    public static final double k_elevatorkP = 0.3;
    public static final double k_elevatorkI = 0.1;
    public static final double k_elevatorkD = 0.15;
    public static final double k_elevatorkS = 0.4;
    public static final double k_elevatorkV = 0.001;
    public static final double k_elevatorkA = 0.001;
    public static final double k_elevatorkG = 0.3;

    public static final double k_wristP = 0.0;
    public static final double k_wristI = 0.0;
    public static final double k_wristD = 0.0;
    public static final double k_wristS = 0.0;
    public static final double k_wristV = 0.0;
    public static final double k_wristG = 0.0;
    
    public static final double k_shooterkP = 0.0;
    public static final double k_shooterkI = 0.0;
    public static final double k_shooterkD = 0.0;
    public static final double k_shooterkS = 0.0;
    public static final double k_shooterkV = 0.0;
    public static final double k_shooterkG = 0.0;
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
    
    public static final double k_elevatorRaiseTime = 0.5;
    public static final double k_elevatorLowerTime = 6.0;
  }

  // Limelight stuff yay
  public static final class LimelightConstants {

    // Tag Reject Distance
    public static final int rejectionDistance = 3;

    // Tag Reject Rotation Rate
    public static final int rejectionRotationRate = 720;
  }
  public static final class VisionConstants {

    // Name
    public static final String k_limelightName = "limelight-three";

    // PID for Tag Relative Control in General
    public static final double kP_aim = 0.04;
    public static final double kI_aim = 0.000;
    public static final double kD_aim = 0.000;

    public static final double kP_range = 0.13;
    public static final double kI_range = 0.0;
    public static final double kD_range = 0.0;

    public static final double kP_strafe = 0.13;
    public static final double kI_strafe = 0.0;
    public static final double kD_strafe = 0.0;

    // Aim/Range
    public static final double k_aimThreshold = 0.5;
    public static final double k_rangeThresholdMax = -3.8;
    public static final double k_rangeThresholdMin = -4.2;
    public static final double k_rangeTarget = -4;

    // AimNRange Reef Right
    public static final double k_aimReefRightThresholdMax = 0.5;
    public static final double k_aimReefRightThresholdMin = -0.5;
    public static final double k_aimReefRightTarget = 0;

    public static final double k_rangeReefRightThresholdMax = -0.63;
    public static final double k_rangeReefRightThresholdMin = -0.68;
    public static final double k_rangeReefRightTarget = -0.65;

    public static final double k_strafeReefRightThresholdMax = 0.20;
    public static final double k_strafeReefRightThresholdMin = 0.15;
    public static final double k_strafeReefRightTarget = 0.18;

    // AimNRange Reef Left
    public static final double k_aimReefLeftThresholdMax = 0.5;
    public static final double k_aimReefLeftThresholdMin = -0.5;
    public static final double k_aimReefLeftTarget = 0;

    public static final double k_rangeReefLeftThresholdMax = -0.63;
    public static final double k_rangeReefLeftThresholdMin = -0.68;
    public static final double k_rangeReefLeftTarget = -0.65;

    public static final double k_strafeReefLeftThresholdMax = -0.15;
    public static final double k_strafeReefLeftThresholdMin = -0.20;
    public static final double k_strafeReefLeftTarget = -0.18;

    public static final double k_tzValidRange = -1.5;
    public static final double k_yawValidRange = 35;
  }
}
