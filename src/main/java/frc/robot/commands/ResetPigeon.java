package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetPigeon extends Command{
    
    SwerveSubsystem subsystem;
    Pigeon2 pigeon;

    public ResetPigeon(Pigeon2 pigeon, SwerveSubsystem subsystem){
        this.pigeon = pigeon;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        pigeon.reset();
        subsystem.swerveDrive.zeroGyro();
        System.out.println("Pigeon Resetado");
    }
    
    @Override
    public boolean isFinished() {
        return pigeon.getYaw().getValueAsDouble() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
