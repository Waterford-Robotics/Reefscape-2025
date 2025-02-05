// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.Limelight.Localization;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// Swerve Subsystem Code yippee
public class SwerveSubsystem extends SubsystemBase {

  // Imports stuff from the JSON Files
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  public static SwerveDrive swerveDrive;

  // Red Alliance sees forward as 180 degrees, Blue Alliance sees as 0
  public static int AllianceYaw;

  // Field2d
  private Field2d m_field = new Field2d();

  // Creates a New SwerveSubsystem
  public SwerveSubsystem() {
    
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    // TURN OFF DURING COMPETITION BECAUSE IT * WILL *  SLOW YOUR ROBOT (It's for displaying info in Shuffleboard)
    //SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    // Initializes robot using the JSON Files with all the constants so you don't have to. Hooray!
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.k_maxSpeed);
    } 
      catch (Exception e) {
      throw new RuntimeException(e);
    }
    
    // Cosine Compensator makes your robot slower on some wheels. Set it to opposite bool if it drives funky
    swerveDrive.setCosineCompensator(false);

    // Keeps robot locked in position when moving, keep false
    swerveDrive.setHeadingCorrection(false);

    // Field on Smart Dashboard
    SmartDashboard.putData("Field", m_field);

    // Configure the PathPlanner Stuff
    AutoBuilder.configureHolonomic( 
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      AutoConstants.k_autoConfig,
      () -> {

        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {  
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  // This method will be called once per scheduler run
  // Periodically update the odometry w/vision
  public void periodic() {

    // Updates Odometry with Vision if Applicable
    updateVisionMeasurements();

    // Update Field2d
    m_field.setRobotPose(swerveDrive.getPose());

    // Puts position of robot on smartdashboard
    SmartDashboard.putNumber("X Position", swerveDrive.getPose().getX());
    SmartDashboard.putNumber("Y Position", swerveDrive.getPose().getY());
  }

  // Command to drive the robot using translative values and heading as angular velocity.
  // translationX - Translation in the X direction. Cubed for smoother controls.
  // translationY - Translation in the Y direction. Cubed for smoother controls.
  // angularRotationX - Angular velocity of the robot to set. Cubed for smoother controls.
  // Returns Drive command.

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumVelocity()), 0.8),
        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
        true,
        false);
    });
  }

  // Command to drive the robot using translative values and heading as a setpoint.
  // translationX Translation in the X direction. Cubed for smoother controls.
  // translationY Translation in the Y direction. Cubed for smoother controls.
  // headingX     Heading X to calculate angle of the joystick.
  // headingY     Heading Y to calculate angle of the joystick.
  // returns Drive command.
  // I'm kinda useless btw, thanks for reading
  
  public Command driveCommandTranslative(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(), scaledInputs.getY(),
        headingX.getAsDouble(), headingY.getAsDouble(),
        swerveDrive.getOdometryHeading().getRadians(),
        swerveDrive.getMaximumVelocity()
      ));
    });
  }

  // Gets the current pose (position and rotation) of the robot, as reported by odometry.
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  // Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
  // method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
  // keep working.
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  // Updates Odometry with the Limelight Readings using MT2 - Old, use updateVisionMeasurements()
  public void updateVisionOdometry() {

    // Used to stop updating upon condiditions
    boolean doRejectUpdate = false;

    // Setting Yaw to Compensate for Red Alliance Limelight Localization
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){
      if (alliance.get() == DriverStation.Alliance.Red) {
        AllianceYaw = 180;
      }
      else if (alliance.get() == DriverStation.Alliance.Blue){
        AllianceYaw = 0;
      }
    }

    // Gets the robot's yaw for LL, then gets a field pose estimate using MT2
    // IMPORTANT: LOOK AT THE NOTE ABOVE FOR THE ALLIANCE YAW VARIABLE!!!
    LimelightHelpers.SetRobotOrientation("", swerveDrive.getYaw().getDegrees() + AllianceYaw, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    
    // If angular velocity is greater than 720 deg/s, ignore vision updates
    if (Math.abs(swerveDrive.getGyro().getRate()) > 720) {
      doRejectUpdate = true;
    }

    // If there are no tags in sight, ignore vision updates
    if (mt2Estimate.tagCount == 0 || mt2Estimate == null) {
      doRejectUpdate = true;
    }

    // If all conditions are met, update vision
    if (!doRejectUpdate && mt2Estimate != null) {
      swerveDrive.addVisionMeasurement(mt2Estimate.pose, mt2Estimate.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
    }
  }

  // Check if pose estimate is valid
  private boolean poseEstimateIsValid(LimelightHelpers.PoseEstimate estimate) {
    return Math.abs(swerveDrive.getGyro().getRate()) < LimelightConstants.rejectionRotationRate
      && estimate.avgTagDist < LimelightConstants.rejectionDistance;
  }

  // Updates Odometry with the Limelight Readings using MT2 - Replacement for updateVisionOdometry()
  public void updateVisionMeasurements() {

    // Setting Yaw to Compensate for Red Alliance Limelight Localization
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){
      if (alliance.get() == DriverStation.Alliance.Red) {
        AllianceYaw = 180;
      }
      else if (alliance.get() == DriverStation.Alliance.Blue){
        AllianceYaw = 0;
      }
    }

    // For each limelight...
    for (Localization.LimelightPoseEstimateWrapper estimateWrapper : Localization.getPoseEstimates(swerveDrive.getYaw().getDegrees())) {

      // If there is a tag in view and the pose estimate is valid...
      if (estimateWrapper.tiv && poseEstimateIsValid(estimateWrapper.poseEstimate)) {

        // Add the vision measurement to the swerve drive
        swerveDrive.addVisionMeasurement(estimateWrapper.poseEstimate.pose,
          estimateWrapper.poseEstimate.timestampSeconds,
          estimateWrapper.getStdvs(estimateWrapper.poseEstimate.avgTagDist));

        // Update position on Field2d
        estimateWrapper.field.setRobotPose(estimateWrapper.poseEstimate.pose);
        m_field.setRobotPose(estimateWrapper.field.getRobotPose());
      }
    }
  }

  // Very big and scary follow path command. Pray it works, because it's massive.
  public void followPathAutobuilderCommand(String pathName) {
    try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        AutoBuilder.followPath(path).schedule();;
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  // Gets the current velocity (x, y and omega) of the robot
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  // Set chassis speeds with closed-loop velocity control.
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  // Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  // Gets robot yaw
  public Rotation2d getYaw() {
    return swerveDrive.getYaw();
  }

  // Gets robot rotation rate in degrees
  public double getRotationRateDegrees() {
    return swerveDrive.getGyro().getRate();
  }

  // Sets X Pose to modules
  public void setx() {
    swerveDrive.lockPose();
  }

  // Drive Field Oriented With ChassisSpeeds
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  // Get Auto Command
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  // Returns the swerve drive
  public static SwerveDrive getInstance() {
    return swerveDrive;
  }
}
