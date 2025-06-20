// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.autonomous.AutonomousInterface;
import frc.robot.config.constants.ConstantDevices.CAN;
import frc.robot.config.constants.ConstantDevices.DIO;
import frc.robot.config.constants.ConstantDevices.DUTY_CYCLE;
import frc.robot.config.constants.ConstantDevices.CONTROLLERS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer robotContainer;
  
  // Variaveis do elevador
  static double KpElev = 0.01;
  static double KiElev = 0;
  static double KdElev = 0;
  
  double tolerancia_elevador=30.0;
  boolean zerar_elevador = false;
  boolean seguranca_elevador = false;
  
  double angulo_elevador;
  double output_elevador;
  public static double setpoint_elevador;

  // Autonomous
  AutonomousInterface autonomoConfig;

  // Variaveis do intake
  double output_intake;
  double medida_inicial;
  double speed_intake;
  double angulo_intake;

  static double Kp_intake = 0.01;
  static double Ki_intake = 0;
  static double Kd_intake= 0;
  public static double setpoint_intake = 60;
  
  boolean trava_motor_bola = false;
  boolean tem_bola_intake = false;

  DutyCycleEncoder encoder_intake = new DutyCycleEncoder(DUTY_CYCLE.ENCODER_INTAKE);
  SparkMax motor_intake = new SparkMax(CAN.ID.INTAKE_MOTOR_SPARK, SparkMax.MotorType.kBrushless);
  public static SparkMax motor_da_bola = new SparkMax(CAN.ID.ALGAE_MOTOR_SPARK, SparkMax.MotorType.kBrushless);

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //variaveis de log
  double lasttick;

  //variaveis para maquina de estado
  int estado_atual= 0;
  double setpoint_3_intake = 0;
  double setpoint_1_intake = 0;
  double setpoint_2_intake = 0;
  double setpoint_1_elevador = 0;
  double setpoint_2_elevador = 0;
  double setpoint_3_elevador = 0;
  boolean hablita_maquina_estados = false;

  // Controles
  Timer timer = new Timer();
  int vezesBotaoApertado = 0;
  Joystick joystick = new Joystick(CONTROLLERS.INTAKE_CONTROLLER_PORT);

  // Elevador
  Encoder encoderElev = new Encoder(DIO.ELEVATOR_ENCODER_A, DIO.ELEVATOR_ENCODER_B);
  SparkMax Elevador15 = new SparkMax(CAN.ID.LEFT_ELEVATOR_SPARK, SparkMax.MotorType.kBrushless);
  SparkMax Elevador14 = new SparkMax(CAN.ID.RIGHT_ELEVATOR_SPARK, SparkMax.MotorType.kBrushless);

  // Intake
  DigitalInput fimDeCurso_Cima = new DigitalInput(DIO.LIMIT_SWITCH_UP);
  DigitalInput fimDeCurso_baixo = new DigitalInput(DIO.LIMIT_SWITCH_DOWN);
  public static DigitalInput fim_de_curso_Coral = new DigitalInput(DIO.LIMIT_SWITCH_CORAL);

  public static PIDController PIDElevador = new PIDController(KpElev, KiElev, KdElev);
  public static PIDController PID_inatake = new PIDController(Kp_intake, Ki_intake, Kd_intake);
  
  @Override
  public void robotInit() {
    System.out.println("Robo iniciado\n");
    setpoint_elevador = 0.0;
    setpoint_intake = 58.0;
    
    robotContainer = new RobotContainer();
    autonomoConfig = new AutonomousInterface();

    encoderElev.setDistancePerPulse(360.0/2048.0);
    encoder_intake.setDutyCycleRange(0, 360);
    encoderElev.setReverseDirection(true);
    
    PIDElevador.setTolerance(tolerancia_elevador);
    PID_inatake.setTolerance(4.0);
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();  
  }

  @Override
  public void autonomousInit() {
    robotContainer.setMotorBrake(true);
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    System.out.println("Teleop iniciado\n");

    timer.reset();
    timer.start();
    encoderElev.reset();
    
    vezesBotaoApertado = 0;
    lasttick = 0.0;
    estado_atual = 0;
    setpoint_intake = 58.0;
    setpoint_elevador = 0.0;
    tem_bola_intake = false;
    trava_motor_bola = false;
    hablita_maquina_estados = false;
  }
  
  @Override
  public void teleopPeriodic() {
    if(!hablita_maquina_estados) {
      if(joystick.getRawButton(1)){//L1
        setpoint_1_elevador=setpoint_elevador;
        setpoint_1_intake = 68.0;
        setpoint_2_elevador = 0;
        setpoint_2_intake = 68.0;
        setpoint_3_elevador = 0.0;
        setpoint_3_intake = 55.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
        motor_da_bola.set(0.1);
        vezesBotaoApertado++;
        trava_motor_bola=true;
      }

      if(joystick.getRawButton(2)){//L2
        setpoint_1_elevador=0.0;
        setpoint_1_intake = 72.0;
        setpoint_2_elevador = 210.0;
        setpoint_2_intake = 72.0;
        setpoint_3_elevador = 210.0;
        setpoint_3_intake = 72.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }
      
      if(joystick.getRawButton(3)){//L3
        setpoint_1_elevador=0;
        setpoint_1_intake = 72.0;
        setpoint_2_elevador = 769.0;
        setpoint_2_intake = 72.0;
        setpoint_3_elevador = 769.0;
        setpoint_3_intake = 72.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }

      if(joystick.getRawButton(4)){//L4
        setpoint_1_elevador=0;
        setpoint_1_intake = 72.0;
        setpoint_2_elevador = 1480.0;
        setpoint_2_intake = 72.0;
        setpoint_3_elevador = 1480.0;
        setpoint_3_intake = 92.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }

      if(joystick.getRawButton(8)){//ir para L2 da bola
        setpoint_1_elevador=624.0;
        setpoint_1_intake = 225.0;
        setpoint_2_elevador = 624.0;
        setpoint_2_intake = 225.0;
        setpoint_3_elevador = 624.0;
        setpoint_3_intake = 225.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }

      if(joystick.getRawButton(9)){//ir para L3 da bola
        setpoint_1_elevador=1107.0;
        setpoint_1_intake = 225.0;
        setpoint_2_elevador = 1107.0;
        setpoint_2_intake = 225.0;
        setpoint_3_elevador = 1107.0;
        setpoint_3_intake = 225.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }

      if(joystick.getRawButton(7)){//processador
        setpoint_1_elevador=0.0;
        setpoint_1_intake = 225.0;
        setpoint_2_elevador = 0.0;
        setpoint_2_intake = 225.0;
        setpoint_3_elevador = 0.0;
        setpoint_3_intake = 225.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }
    }
    
    if (joystick.getRawButton(6) && vezesBotaoApertado < 2) { // Puxa
      motor_da_bola.set(0.3);
      vezesBotaoApertado++;
      trava_motor_bola = false;
    }

    if (joystick.getRawButton(5) && vezesBotaoApertado < 2) { // Solta
      motor_da_bola.set(-0.3);
      vezesBotaoApertado++;
      trava_motor_bola = false;
    }

    if(vezesBotaoApertado >= 2){ // parar
      vezesBotaoApertado = 0;
      motor_da_bola.set(0);
    }
    
    // Controle do elevador
    angulo_elevador = encoderElev.getDistance();
    output_elevador = PIDElevador.calculate(angulo_elevador, setpoint_elevador);
  
    // Fim de curso de cima
    if(fimDeCurso_Cima.get()) {
      if(output_elevador>0) {
        output_elevador=0;
      }
      
      if(setpoint_elevador>1480) {
        setpoint_elevador=1480.0;
      }
    }
    
    //ler fim de curso de baixo
    if(fimDeCurso_baixo.get()) {
      if(output_elevador<0) {
        output_elevador=0;
      }
      
      if(setpoint_elevador<0.0) {
        setpoint_elevador=0.0;
      }

      if(zerar_elevador == true && (angulo_elevador > tolerancia_elevador || angulo_elevador < -1 * tolerancia_elevador)) {
        zerar_elevador=false;  
      }
    } else {
      zerar_elevador=true;
    }
  
    // Segurança do elevador em relação ao intake
    if(seguranca_elevador) {
      output_elevador=0.0;
    }
    
    Elevador14.set(output_elevador);
    Elevador15.set(-output_elevador);
    
    // Inicio do código do intake
    angulo_intake = encoder_intake.get() * 360.0;
    output_intake = PID_inatake.calculate(angulo_intake, setpoint_intake);
  
      // Ler posição minima do intake
      if(angulo_intake < 55) {
        if(output_intake<0) {
          output_intake=0.0;
        }

        if(setpoint_intake < 55){
          setpoint_intake = 55.0;
        }
      }
  
      // Ler posição maxima do intake 
      if(angulo_intake > 230) {
        if(output_intake > 0) {
          output_intake=0.0;
        }

        if (setpoint_intake > 230) {
          setpoint_intake = 230.0;
        }
      }

    
    motor_intake.set(output_intake);
  
    if (angulo_intake < 64) {
      seguranca_elevador = true;
    } else{
      seguranca_elevador = false;
    }  

    // Motor da bola e do coral
    if(trava_motor_bola) {
      motor_da_bola.set(0.2);
    }

    if(!fim_de_curso_Coral.get() && trava_motor_bola){
      motor_da_bola.set(0);
      trava_motor_bola=false;
    }
      
    
    // Inicio do código da maquina de estados 
    if(hablita_maquina_estados) {
      switch(estado_atual) {
        case 1: {
          setpoint_elevador=setpoint_1_elevador;
          setpoint_intake=setpoint_1_intake;
          if(PIDElevador.atSetpoint() && PID_inatake.atSetpoint()) {
            estado_atual = 2;
            setpoint_elevador=setpoint_2_elevador;
            setpoint_intake=setpoint_2_intake;
          }
        } break;
        case 2: {
          if(PIDElevador.atSetpoint() && PID_inatake.atSetpoint()) {
            estado_atual = 3;
            setpoint_elevador=setpoint_3_elevador;
            setpoint_intake=setpoint_3_intake;
          }
        } break;
        case 3: {
          if(PIDElevador.atSetpoint() && PID_inatake.atSetpoint()) {
            hablita_maquina_estados = false;
          }
        } break;
      }
    }

    // Imprimir os logs
    if(timer.get() > (lasttick + 2.0)) {
      lasttick = timer.get();
      System.out.printf("Set_point elevador: %2f\n", setpoint_elevador);
      System.out.printf("Angulo_elevador: %2f\n", angulo_elevador);
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

}
