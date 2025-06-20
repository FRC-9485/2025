package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;

public class Turn extends Command {
    private double setpoint;
    private double pigeonSetpoint;
    private final Pigeon2 pigeon2;
    private final SwerveSubsystem subsystem;

    private static final double kP_ROTATION = 0.05;
    private static final double kI_ROTATION = 0.2;
    private static final double kD_ROTATION = 0.02;
    
    private XboxController xboxController;
    private static Timer timer = new Timer();
    private static final double MAX_SPEED = 7.0;
    private final Translation2d translation2d = new Translation2d(0, 0);
    private final PIDController controller = new PIDController(kP_ROTATION, kI_ROTATION, kD_ROTATION);
    
    public Turn(SwerveSubsystem subsystem, double setpoint, XboxController controle, Pigeon2 pigeon) {
        addRequirements(subsystem);
        this.pigeon2 = pigeon;
        this.setpoint = setpoint;
        this.subsystem = subsystem;
        this.xboxController = controle;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        controller.setTolerance(2.0); 
        controller.enableContinuousInput(-180, 180); 
        System.out.printf("Iniciando rotação para %f graus", setpoint);
        pigeonSetpoint = pigeon2.getYaw().getValueAsDouble() % 360.0000 + setpoint;
    }

    @Override
    public void execute() {
        double currentAngle = pigeon2.getYaw().getValueAsDouble();
        double rotationSpeed = controller.calculate(currentAngle, setpoint);

        if (xboxController.getRightX() != 0 || xboxController.getRightY() != 0) {
            this.cancel();
        }

        System.out.println("Valor divido" + pigeon2.getYaw().getValueAsDouble() % 360);
        System.out.println("Valor divido e somado: " + pigeonSetpoint);

        if (rotationSpeed <= -45) {
            rotationSpeed = -45;
            subsystem.drive(new Translation2d(0,0), 0, true);
        } else if(rotationSpeed >= 45){
            rotationSpeed = 45;
            subsystem.drive(translation2d, 0, true);
        }
        // Limita a velocidade de rotação
        rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_SPEED), MAX_SPEED);
        
        System.out.println("Ângulo atual: " + currentAngle);
        System.out.println("Velocidade de rotação: " + -rotationSpeed);
        
        subsystem.drive(translation2d, -rotationSpeed, true);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint() || timer.get() >= 3.0;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.drive(translation2d, 0, true);
        System.out.println("Rotação concluída");
    }
}
