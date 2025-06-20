package frc.robot.config.constants;

public class ConstantDevices {

    public static class CAN {
        public static class ID {
            public static final int PIGEON2 = 9;
            public static final int RIGHT_ELEVATOR_SPARK = 14;
            public static final int LEFT_ELEVATOR_SPARK = 15;
            public static final int INTAKE_MOTOR_SPARK = 17;
            public static final int ALGAE_MOTOR_SPARK = 18;
        }
    }
    
    public static class DIO {
        public static final int ELEVATOR_ENCODER_A = 6;
        public static final int ELEVATOR_ENCODER_B = 8;
        public static final int LIMIT_SWITCH_UP = 2;
        public static final int LIMIT_SWITCH_DOWN = 3;
        public static final int LIMIT_SWITCH_CORAL = 0;
    }

    public static class DUTY_CYCLE {
        public static final int ENCODER_INTAKE = 1;
    }
    
    public static class CONTROLLERS {
        public static final double DRIVER_DEADBAND = 0.1;

        public static final int DRIVER_CONTROLER_PORT = 0;
        public static final int INTAKE_CONTROLLER_PORT = 1;
    }

    public static class NETWORK {
        public static final String LIMELIGHT_TABLE_NAME = "Limelight";
    }
}
