package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorPIDConstants;

public class WristSubsystem extends SubsystemBase{

  private TalonFX m_wrist;
  private TalonFX m_shooter;

  private TalonFXConfiguration wristConfig;
  private TalonFXConfiguration shooterConfig;

  private Measure<Angle> lastDesiredPosition;

  public WristSubsystem() {
    
    m_wrist = new TalonFX(MotorIDConstants.k_wristKrakenID, "Elevator/Coral");
    m_shooter = new TalonFX(MotorIDConstants.k_shooterKrakenID, "Elevator/Coral");

    lastDesiredPosition = Units.Rotations.of(0);

    wristConfig = new TalonFXConfiguration();
    wristConfig.Slot0.kP = MotorPIDConstants.k_wristP;
    wristConfig.Slot0.kI = MotorPIDConstants.k_wristI;
    wristConfig.Slot0.kD = MotorPIDConstants.k_wristD;
    wristConfig.Slot0.kS = MotorPIDConstants.k_wristS;
    wristConfig.Slot0.kV = MotorPIDConstants.k_wristV;
    wristConfig.Slot0.kG = MotorPIDConstants.k_wristG;

    wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristConfig.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_supplyCurrentLimit;
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
    wristConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(0.7).in(Units.Rotations); // TODO: Check me
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(2).in(Units.Rotations); // Starting position
    wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    wristConfig.Feedback.SensorToMechanismRatio = 0.4545;

    shooterConfig = new TalonFXConfiguration();

    // PID Stuff (Shooter)
    shooterConfig.Slot0.kP = MotorPIDConstants.k_shooterkP;
    shooterConfig.Slot0.kI = MotorPIDConstants.k_shooterkI;
    shooterConfig.Slot0.kD = MotorPIDConstants.k_shooterkD;
    shooterConfig.Slot0.kS = MotorPIDConstants.k_shooterkS;
    shooterConfig.Slot0.kV = MotorPIDConstants.k_shooterkV;

    // Kraken Configs
    shooterConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorConstants.k_shooterRampRate;
    shooterConfig.MotorOutput.PeakForwardDutyCycle = MotorConstants.k_shooterClosedMaxSpeed;
    shooterConfig.MotorOutput.PeakReverseDutyCycle = -MotorConstants.k_shooterClosedMaxSpeed;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = MotorConstants.k_supplyCurrentLimit;

    m_wrist.getConfigurator().apply(wristConfig, 0.05);
    m_shooter.getConfigurator().apply(shooterConfig, 0.05);

    m_wrist.setInverted(true);
  }

    public Measure<Angle> getWristPosition(){
      return Units.Rotations.of(m_wrist.get());
    }

    public void setPosition(Measure<Angle> angle){
      m_wrist.setControl(new PositionVoltage(angle.in(Units.Rotations)));
      lastDesiredPosition = angle;
    }

    public void setNeutral() {
      m_wrist.setControl(new NeutralOut());
    }

    public void resetSensorPosition(Measure<Angle> setpoint) {
      m_wrist.setPosition(setpoint.in(Units.Rotations));
    }

    public void shoot(){
      m_shooter.set(0.1);
    }

    public void stopShooter(){
      m_shooter.set(0);
    }

    public void intake(){
      // m_wrist.setPosition(WristConstants.k_coralIntakeHeight.in(Units.Rotations));
      m_shooter.set(-0.1);
    }

    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Wrist/Pos", m_wrist.getPosition().getValueAsDouble());
      SmartDashboard.putString("Wrist/Units", m_wrist.getPosition().getUnits());
      SmartDashboard.putNumber("Wrist/CLO", m_wrist.getClosedLoopOutput().getValueAsDouble());
      SmartDashboard.putNumber("Wrist/Output", m_wrist.get());
      SmartDashboard.putNumber("Wrist/Inverted", m_wrist.getAppliedRotorPolarity().getValueAsDouble());
      SmartDashboard.putNumber("Wrist/Current", m_wrist.getSupplyCurrent().getValueAsDouble());
  }
}
