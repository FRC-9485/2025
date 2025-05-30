package frc.robot.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class PathPlanner {
    
    public void configurarIntake(){
        HashMap<String, Command> hashMap = new HashMap<>();

        hashMap.put("intake", new InstantCommand(() ->{
            Robot.setpoint_intake = 72.0;
        }));
    }
}
