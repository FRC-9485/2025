package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.GenericEntry;

public class AutonomoInterface {
    private ShuffleboardTab tab;
    private SendableChooser<String> posicaoChooser;
    private SendableChooser<String> alvoChooser;
    private GenericEntry statusWidget;

    public AutonomoInterface() {
        // Cria uma nova aba no Shuffleboard
        tab = Shuffleboard.getTab("Autônomo");
        
        // Cria os seletores
        posicaoChooser = new SendableChooser<>();
        posicaoChooser.setDefaultOption("Centro", "CENTRO");
        posicaoChooser.addOption("Esquerda", "ESQUERDA");
        posicaoChooser.addOption("Direita", "DIREITA");
        
        alvoChooser = new SendableChooser<>();
        alvoChooser.setDefaultOption("Speaker", "SPEAKER");
        alvoChooser.addOption("Amp", "AMP");
        alvoChooser.addOption("Apenas Sair", "SAIR");
        
        // Adiciona os widgets ao Shuffleboard
        tab.add("Posição Inicial", posicaoChooser)
           .withSize(2, 1)
           .withPosition(0, 0);
           
        tab.add("Alvo", alvoChooser)
           .withSize(2, 1)
           .withPosition(2, 0);
           
        // Adiciona um widget para mostrar o status
        statusWidget = tab.add("Status", "Pronto")
                         .withSize(2, 1)
                         .withPosition(0, 1)
                         .getEntry();
    }

    public String getPosicaoSelecionada() {
        return posicaoChooser.getSelected();
    }

    public String getAlvoSelecionado() {
        return alvoChooser.getSelected();
    }

    public void atualizarStatus(String status) {
        statusWidget.setString(status);
    }
    public double getElevatorSetpoint(){
        return frc.robot.Robot.setpoint_elevador;
    }
    public double setElevatorSetpoint(double setpoint){
        return frc.robot.Robot.setpoint_elevador = setpoint;
    }
}