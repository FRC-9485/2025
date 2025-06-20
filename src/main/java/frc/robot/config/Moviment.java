package frc.robot.config;

import swervelib.parser.PIDFConfig;

public class Moviment {
    public class Autonomous {
        public static String TRAJECTORY_1_NAME = "";
        public static String CURRENT_TRAJECTORY_NAME = "";
        public static final String DEFAULT_TRAJECTORY_NAME = "";
        public static final String[] TRAJECTORYS = {"1", "2", "3"};

        public static final class PID {
            public static final PIDFConfig AUTONOMUS_PID_Y = new PIDFConfig(0.6, 0, 0);
            public static final PIDFConfig AUTONOMUS_PID_X = new PIDFConfig(0.5, 0, 0.05);
            public static final PIDFConfig AUTONOMUS_PID_ANGLE = new PIDFConfig(0.1, 0, 0.01);
        }
    }
    
    public class FiledOrientation {
        public static boolean isRedAlliance = false;
    }
}
