// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * Classe de constantes
 */
public final class Constants {  
  public static final class Dimensoes {
    // Tempo de loop (sparkMax + normal = 130ms)
    public static final double LOOP_TIME = 0.13;
    // Massa do robô
    public static final double ROBOT_MASS = 38;
    //Velocidade máxima
    public static double MAX_SPEED = 7.0;

    //Posições do centro de massa
    private static final double xMass = 0;
    private static final double yMass = 0;
    private static final double zMass = 0.8;

    // Centro de massa do chassi
    public static final Matter CHASSIS    = new Matter(new Translation3d(xMass, yMass, (zMass)), ROBOT_MASS);
   }

    // Classe que contém os PID para o autônomo
        
    
    public static final class Tracao {
      // Define se a tração vai ser orientada ao campo (sim = true)
      public static final boolean fieldRelative = true;
      // false para malha-fechada
      public static final boolean isOpenLoop = false;
      // true para correção de aceleração
      public static final boolean accelCorrection = false;
      // constante para diminuir o input do joystick (0 < multiplicadorRotacional <= 1)
      public static final double multiplicadorRotacional = 0.8;
      // constante para diminuir o input do joystick (y)
      public static final double multiplicadorTranslacionalY = 0.7;
      // constante para diminuir o input do joystick (x)
      public static final double multiplicadorTranslacionalX = 0.7;

      public static final double TURN_CONSTANT = 0.75;

      // constante que define a velocidade máxima
      public static double MAX_SPEED = 7.0;

      public static final double dt = 0.02;

      public static final double constantRotation = 4;
    }

}
