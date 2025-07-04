// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import frc.robot.config.variables.VariableMoviment.DRIVE;
import frc.robot.config.constants.ConstantDevices.CAN;
import frc.robot.config.constants.ConstantMoviment.SWERVE;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.SwerveController;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  public SwerveDrive swerveDrive;
  Pigeon2 pigeon = new Pigeon2(CAN.ID.PIGEON2);

  // Método construtor da classe
  public SwerveSubsystem(File directory) {
      // Seta a telemetria como nível mais alto
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

      // Acessa os arquivos do diretório .JSON
      try {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(SWERVE.MAX_SPEED);
      } catch (Exception e) {
        throw new RuntimeException(e);
      }

      swerveDrive.setChassisDiscretization(SWERVE.ENABLE_CHASSIS_DISCRETIZATION, 0.2); 
      setupPathPlanner();
  }
    
  @Override
  public void periodic() {
    // Dentro da função periódica atualizamos nossa odometria
    swerveDrive.updateOdometry();
  }

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (SWERVE.ENABLE_FEED_FORWARD) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
              );
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(0.2, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(0.05, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              boolean currentAlliace = alliance.get() == DriverStation.Alliance.Red;
              DRIVE.IS_RED_ALLIANCE = currentAlliace;
              return currentAlliace;
            }
            return false;
          },
          this
        );
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  //Movimenta o robô com o joystick esquerdo, e mira o robo no ângulo no qual o joystick está apontando
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY) {
    return run(() -> {
      // Faz o robô se mover
      double xInput = Math.pow(translationX.getAsDouble(), 1); 
      double yInput = Math.pow(translationY.getAsDouble(), 1); 
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      //swerveDrive.getYaw().getRadians(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  //Movimenta o robô com o joystick esquerdo, e gira o robô na intensidade na qual o joystick direito está para o lado
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
      return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 1); 
      double yInput = Math.pow(translationY.getAsDouble(), 1); 
      // Faz o robô se mover
      swerveDrive.driveFieldOriented(new ChassisSpeeds(xInput*swerveDrive.getMaximumChassisVelocity(),
                                                      yInput*swerveDrive.getMaximumChassisVelocity(),
                                                      angularRotationX.getAsDouble()*swerveDrive.getMaximumChassisAngularVelocity()));
    }); 
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

    // Função drive que chamamos em nossa classe de comando Teleoperado
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
      swerveDrive.drive(translation, rotation, fieldRelative, true);
    }

    // Função para obter a velocidade desejada a partir dos inputs do gamepad
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
      return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, 
      getHeading().getRadians());
    }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) {
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 0, 0, 0, SWERVE.MAX_SPEED);
  }

  // Função que retorna a posição do robô (translação e ângulo), (Usado no autônomo)
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }
  
  // Retorna a velocidade relativa ao campo
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  // Retorna a configuração do swerve
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  // Retorna o objeto de controle, o qual é usado para acessar as velocidades máximas por exemplo
  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(pigeon.getYaw().getValueAsDouble(), 0));
  }

  // Reseta a odometria para uma posição indicada (Usado no autônomo)
  public void resetOdometry(Pose2d posicao) {
    swerveDrive.resetOdometry(posicao);
  }

  // Seta a velocidade do chassi (Usado no autônomo)
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
}
