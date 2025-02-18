package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    Spark blinkin;

    public LEDSubsystem(){
        blinkin = new Spark(LEDConstants.blinkinPort);
    }

    public void periodic(){
        setConfetti();
    }

    public void setRainbowParty() {
        blinkin.set(-0.97);
    }

    public void setConfetti() {
        blinkin.set(-0.87);
    }

    public void setBpmOcean() {
        blinkin.set(-0.65);
    }

    public void setFireLarge() {
        blinkin.set(-0.57);
    }

    public void turnOff() {
        blinkin.set(0);
    }

    public void setColorWavesLava() {
        blinkin.set(-0.34);
    }

    public void setE2EB2B() {
        blinkin.set(-0.03);
    }

    public void setSparkle221() {
        blinkin.set(0.39);
    }

    public void setTwinkleForest() {
        blinkin.set(-0.47);
    }
}
