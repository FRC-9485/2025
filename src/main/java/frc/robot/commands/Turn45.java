package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class Turn45 extends Command {
    // Constantes para ajuste fino
    private static final double kP_ROTATION = 0.05; // Reduzido para movimento mais suave
    private static final double kI_ROTATION = 0.2;
    private static final double kD_ROTATION = 0.02;
    double setpoint;
    double mySetpoint;
    
    private final SwerveSubsystem subsystem;
    private final Translation2d translation2d = new Translation2d(0, 0);
    private final PIDController controller = new PIDController(kP_ROTATION, kI_ROTATION, kD_ROTATION);
    private final Pigeon2 pigeon2 = new Pigeon2(9);
    private static final double MAX_SPEED = 7.0; // Reduzido para maior controle
    private static Timer timer = new Timer();
    private XboxController xboxController;
    
    public Turn45(SwerveSubsystem subsystem, double setpoint, RobotContainer container) {
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.setpoint = setpoint;
        this.xboxController = container.controleXbox;
    }

    @Override
    public void initialize() {
        controller.setTolerance(2.0); 
        controller.enableContinuousInput(-180, 180); 
        System.out.println("Iniciando rotação para -45 graus");
        mySetpoint = pigeon2.getYaw().getValueAsDouble() % 360.0000 + setpoint;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentAngle = pigeon2.getYaw().getValueAsDouble();
        double rotationSpeed = controller.calculate(currentAngle, setpoint);

        if (xboxController.getRightX() != 0 || xboxController.getRightY() != 0) {
            this.cancel();
        }

    //    if (xboxController.getLeftX() > 0 || xboxController.getLeftX() < 0 || xboxController.getLeftY() > 0 || xboxController.getLeftY() < 0) {
    //        System.out.println("Parando de curvar");
    //        controller.calculate(currentAngle, currentAngle);
    //        System.out.println("Curva parada");
    //     }

        System.out.println("Valor divido" + pigeon2.getYaw().getValueAsDouble() % 360);
        //double rotationSpeed = controller.calculate(currentAngle, setpoint);
        System.out.println("Valor divido e somado: " + mySetpoint);

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
        // Removido o reset do pigeon2 para manter a referência
        System.out.println("Rotação concluída");
    }
}
