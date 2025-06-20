package frc.robot.config.constants;

import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

public class ConstantMoviment {
    public static final class PID {
        public static final PIDFConfig AUTONOMUS_PID_Y = new PIDFConfig(0.6, 0, 0);
        public static final PIDFConfig AUTONOMUS_PID_X = new PIDFConfig(0.5, 0, 0.05);
        public static final PIDFConfig AUTONOMUS_PID_ANGLE = new PIDFConfig(0.1, 0, 0.01);
    }

    public static final class DIMENSIONS {
        public static double MAX_SPEED = 7.0;
        public static final double ROBOT_MASS = 38;
        public static final double LOOP_TIME = 0.13; // SparkMax + Normal

        //Posições do centro de massa
        private static final double xMass = 0;
        private static final double yMass = 0;
        private static final double zMass = 0.8;
        public static final Matter CHASSIS = new Matter(new Translation3d(xMass, yMass, (zMass)), ROBOT_MASS);
   }

    public static final class SWERVE {
        public static final double MAX_SPEED = 7.0;
        public static final boolean IS_OPEN_LOOP = false;
        public static final boolean IS_FIELD_RELATIVE = true;
        public static final boolean IS_ACELL_CORRECTION = false;

        public static final boolean ENABLE_CHASSIS_DISCRETIZATION = true;
        public static final boolean ENABLE_FEED_FORWARD = true;
    
        public static final double DT = 0.02;
        public static final double ROTATION_CONSTANT = 4;

        public static final double TURN_CONSTANT = 0.75;
        public static final double ROTATIONAL_MULTIPLIER = 0.8;
        public static final double TRASNLATIONAL_MULTIPLIER_X = 0.7;
        public static final double TRASNLATIONAL_MULTIPLIER_Y = 0.7;
        // constante para diminuir o input do joystick (0 < multiplicadorRotacional <= 1) -> Rotation Multplier
        // constante para diminuir o input do joystick (x) -> Trasnlationar Multiplier X
        // constante para diminuir o input do joystick (y) -> Trasnlationar Multiplier Y
    }

}
