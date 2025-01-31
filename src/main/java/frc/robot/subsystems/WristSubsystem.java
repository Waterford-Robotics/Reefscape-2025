package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase{
  private TalonFX m_wrist;
  private TalonFX m_shooter;
  private TalonFXConfiguration krakenConfig;
  private Measure<Angle> lastDesiredPosition;
  public WristSubsystem() {
    
    m_wrist = new TalonFX(MotorIDConstants.k_wristKrakenID);
    m_shooter = new TalonFX(MotorIDConstants.k_shooterKrakenID);
    lastDesiredPosition = Units.Degrees.of(0);

    krakenConfig = new TalonFXConfiguration();
    krakenConfig.Slot0.kP = MotorPIDConstants.k_wristP;
    krakenConfig.Slot0.kI = MotorPIDConstants.k_wristI;
    krakenConfig.Slot0.kD = MotorPIDConstants.k_wristD;
    krakenConfig.Slot0.kS = MotorPIDConstants.k_wristS;
    krakenConfig.Slot0.kV = MotorPIDConstants.k_wristV;
    krakenConfig.Slot0.kG = MotorPIDConstants.k_wristG;

    krakenConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_wristRampRate;
    krakenConfig.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_wristClosedMaxSpeed;
    krakenConfig.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_wristClosedMaxSpeed;
    krakenConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    krakenConfig.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_wristSupplyCurrentLimit;
    krakenConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: Check
    krakenConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
    krakenConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Degrees.of(160).in(Units.Degrees); // TODO: Check me
    krakenConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    krakenConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Degrees.of(20).in(Units.Degrees); // Starting position
    krakenConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    krakenConfig.Feedback.SensorToMechanismRatio = 0.4545;

    m_wrist.getConfigurator().apply(krakenConfig, 0.05);
  }

    public Measure<Angle> getWristPosition(){
      return Units.Degrees.of(m_wrist.get());
    }

    public void setPosition(Measure<Angle> angle){
      m_wrist.setControl(new PositionVoltage(angle.in(Units.Degrees)));
      lastDesiredPosition = angle;
    }

    public void setNeutral() {
      m_wrist.setControl(new NeutralOut());
    }

    public void resetSensorPosition(Measure<Angle> setpoint) {
      m_wrist.setPosition(setpoint.in(Units.Degrees));
    }

    public void shoot(){
      m_shooter.set(0.5);
    }

    public void stopShooter(){
      m_shooter.set(0);
    }

    public void intake(){
      m_wrist.setPosition(WristConstants.k_coralIntakeHeight.in(Units.Degrees));
      m_shooter.set(-0.5);
    }

    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Wrist/Pos", m_wrist.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Wrist/CLO", m_wrist.getClosedLoopOutput().getValueAsDouble());
      SmartDashboard.putNumber("Wrist/Output", m_wrist.get());
      SmartDashboard.putNumber("Wrist/Inverted", m_wrist.getAppliedRotorPolarity().getValueAsDouble());
      SmartDashboard.putNumber("Wrist/Current", m_wrist.getSupplyCurrent().getValueAsDouble());

    // SmartDashboard.putNumber("Wrist/Last Desired Position", (double) lastDesiredPosition.toString()); 
  }
}
