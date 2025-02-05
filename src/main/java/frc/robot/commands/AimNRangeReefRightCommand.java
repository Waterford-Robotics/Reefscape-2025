package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight.LimelightHelpers;


// Positions Robot at the Nearest Valid Target
public class AimNRangeReefRightCommand extends Command {
    
  // Instantiate Stuff
  SwerveSubsystem m_swerveSubsystem;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Controller stuff (woah so many so scary) TODO: TUNE MEEE
  PIDController m_aimController = new PIDController(VisionConstants.kP_aim, VisionConstants.kI_aim, VisionConstants.kD_aim);
  PIDController m_rangeController = new PIDController(VisionConstants.kP_range, VisionConstants.kI_range, VisionConstants.kD_range);
  PIDController m_strafeController = new PIDController(VisionConstants.kP_strafe, VisionConstants.kI_strafe, VisionConstants.kD_strafe);
    
  // All the Valid IDs available for positioning
  int[] validIDs = {6, 7, 8, 9, 10, 11};

  // Valid Tag
  int validTag;

  // Bot Pose Target Space Relative (TX, TY, TZ, Pitch, Yaw, Roll)
  private double[] botPoseTargetSpace = new double[6];


  // Lil boolean for checking for "Tag In View" 
  boolean tiv;

  // Constructor
  public AimNRangeReefRightCommand(SwerveSubsystem driveSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_swerveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // What we do to set up the command TODO: Needs a way to filter out bad poses
  public void initialize() {
    
    // Adds condition that filters out undesired IDs
    LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.k_limelightName, validIDs);

    validTag = (int) LimelightHelpers.getFiducialID(VisionConstants.k_limelightName);

    // Update BotPoseTargetSpace
    botPoseTargetSpace = NetworkTableInstance.getDefault().getTable(VisionConstants.k_limelightName).getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    // Checks for TIV
    tiv = LimelightHelpers.getTV(VisionConstants.k_limelightName) 
      && botPoseTargetSpace[2] > VisionConstants.k_tzValidRange 
      && Math.abs(botPoseTargetSpace[4])< VisionConstants.k_yawValidRange;

    // Timer Reset
    timer.start();
    timer.reset();
  }
    
  // The actual control!
  public void execute() {

    // Update the pose from NetworkTables (Limelight Readings)
    botPoseTargetSpace = NetworkTableInstance.getDefault().getTable(VisionConstants.k_limelightName).getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    // If tags are in view, drive right over!
    if (LimelightHelpers.getFiducialID(VisionConstants.k_limelightName) == validTag) m_swerveSubsystem.driveCommand(() -> limelight_range_PID(), () -> limelight_strafe_PID(), () -> limelight_aim_PID());

    // Otherwise we tell it to quit
    else tiv = false;
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {}

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached 
  public boolean isFinished() {
    return (
      // Range (Distance to Tag)
      botPoseTargetSpace[2] < VisionConstants.k_rangeReefRightThresholdMax
      && botPoseTargetSpace[2]  > VisionConstants.k_rangeReefRightThresholdMin
      
      // Aim (Angle)
      && botPoseTargetSpace[4]  < VisionConstants.k_aimReefRightThresholdMax
      && botPoseTargetSpace[4]  > VisionConstants.k_aimReefRightThresholdMin

      // Strafe (Left Right Positioning)
      && botPoseTargetSpace[0]  < VisionConstants.k_strafeReefRightThresholdMax
      && botPoseTargetSpace[0]  > VisionConstants.k_strafeReefRightThresholdMin)

      // Other quit conditions
      || !tiv || timer.get() > 3;
  }

  // Advanced PID-assisted ranging control with Limelight's TZ value from target-relative data
  private double limelight_range_PID() {

    // Limelight Z Axis Range in meters
    m_rangeController.enableContinuousInput(-3, 0); // TODO: Check these numbers?
    
    // Calculates response based on difference in distance from tag to robot
    double targetingForwardSpeed = m_rangeController.calculate(botPoseTargetSpace[2] - VisionConstants.k_rangeReefRightTarget);

    // Value scale up to robot max speed and invert (double cannot exceed 1.0)
    targetingForwardSpeed *= -1.0 * SwerveConstants.k_maxSpeed;

    // Hooray
    return targetingForwardSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's TX value from target-relative data
  private double limelight_strafe_PID() {

    // Limelight X Axis Range in Meters
    m_strafeController.enableContinuousInput(-3, 3); // TODO: Check me!
    
    // Calculates response based on difference in horizontal distance from tag to robot
    double targetingStrafeSpeed = m_strafeController.calculate(botPoseTargetSpace[0] - VisionConstants.k_strafeReefRightTarget);

    // Value scale up to robot max speed (Double can't exceed 1.0)
    targetingStrafeSpeed *= 1.0 * SwerveConstants.k_maxSpeed;

    // Hooray
    return targetingStrafeSpeed;
  }

  // Advanced PID-assisted ranging control with Limelight's Yaw value from target-relative data
  private double limelight_aim_PID() {

    // Limelight Yaw Angle in Meters
    m_aimController.enableContinuousInput(-30, 30); // TODO: Check Value 
    
    // Calculates response based on difference in angle from tag to robot
    double targetingAngularVelocity = m_aimController.calculate(botPoseTargetSpace[4] - VisionConstants.k_aimReefRightTarget);

    // Multiply by -1 because robot is CCW Positive. Multiply by a reduction 
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingAngularVelocity *= -0.1 * SwerveConstants.kMaxAngularSpeed;

    // Hooray
    return targetingAngularVelocity;
  }
}
