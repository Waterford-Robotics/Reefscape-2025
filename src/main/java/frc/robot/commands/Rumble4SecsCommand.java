package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumble4SecsCommand extends Command{
    double m_run4Secs;
    CommandXboxController m_controller;
    Timer timer = new Timer();
    public Rumble4SecsCommand(double s, CommandXboxController c){
        m_run4Secs = s;
        m_controller = c;
    }
}
