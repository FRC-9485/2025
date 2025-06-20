// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import frc.robot.config.constants.ConstantDevices.CAN;
import frc.robot.config.constants.ConstantDevices.NETWORK;
import frc.robot.config.constants.ConstantDevices.CONTROLLERS;

import frc.robot.config.variables.VariableMoviment.DRIVE;
import frc.robot.config.variables.VariableMoviment.AUTONOMOUS;

import frc.robot.commands.Turn;
import frc.robot.devices.Limelight;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.AlingToTarget;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  Pigeon2 pigeon2 = new Pigeon2(CAN.ID.PIGEON2);
  Limelight limelight = new Limelight(NETWORK.LIMELIGHT_TABLE_NAME);
  public XboxController controleXbox = new XboxController(CONTROLLERS.DRIVER_CONTROLER_PORT);
  private SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
  // Função onde os eventos (triggers) são configurados
  private void configureBindings() {
    new POVButton(controleXbox, 0).whileTrue(new AlingToTarget(limelight, swerve, -2.3, 1.8));
    new POVButton(controleXbox, 90).whileTrue(new AlingToTarget(limelight, swerve, -15.8, 0.9));
    new POVButton(controleXbox, 270).whileTrue(new AlingToTarget(limelight, swerve, 10.4, 1.4));
    
    new JoystickButton(controleXbox, 1).onTrue(new Turn(swerve, -47.0, controleXbox, pigeon2));
    new JoystickButton(controleXbox, 9).onTrue(new ResetPigeon(pigeon2, swerve));
    new JoystickButton(controleXbox, 2).onTrue(new Turn(swerve, 47.0, controleXbox, pigeon2));
    new JoystickButton(controleXbox, 4).onTrue(new Turn(swerve, 2.0, controleXbox, pigeon2));
  }
  
  public RobotContainer() {
    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(setVelocity(1), CONTROLLERS.DRIVER_DEADBAND),
      () -> MathUtil.applyDeadband(setVelocity(2), CONTROLLERS.DRIVER_DEADBAND),
      () ->  MathUtil.applyDeadband(setVelocity(3), CONTROLLERS.DRIVER_DEADBAND))
    );
    
    NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));
    configureBindings();
  }


  // Função que retorna o autônomo
  public Command getAutonomousCommand() {
    // Aqui retornamos o comando que está no selecionador
    return swerve.getAutonomousCommand(AUTONOMOUS.CURRENT_TRAJECTORY_NAME, true);
  }

  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public double setVelocity(int eixo){
    double speed = 0.5;
    double inverter = 1;

    if(controleXbox.getRightBumperButton() && !controleXbox.getLeftBumperButton()){
      speed = 1.0;
    } else if(controleXbox.getLeftBumperButton() && !controleXbox.getRightBumperButton()){
      speed = 0.2;
    } else {
      speed = 0.5;
    }

    if (!DRIVE.IS_RED_ALLIANCE) {
      inverter = -1;
    } else {
      inverter = 1;
    }

    if(eixo == 1){
      return controleXbox.getLeftY() * inverter * speed;
    } else if(eixo == 2){
      return controleXbox.getLeftX() * inverter * speed;
    } else if(eixo == 3){
      return controleXbox.getRightX() * speed;
    }

    return eixo;
  }
}
