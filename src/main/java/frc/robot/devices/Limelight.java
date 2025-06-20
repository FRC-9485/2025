package frc.robot.devices;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight{
    String limelightTableNome;

    public Limelight(String limelightTableName){
        this.limelightTableNome = limelightTableName;
    }

    public NetworkTable limelight() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }
    public boolean getHasTarget(){
        return limelight().getEntry("tv").getDouble(0)==1;
    }
    public double getTx(){
        return limelight().getEntry("tx").getDouble(0.0);
    }
    public double getTy(){
        return limelight().getEntry("ty").getDouble(0.0);
    }
    public boolean setLedMode(int mode){
        return limelight().getEntry("ledMode").setNumber(mode);
    }
    public double getTagId(){
        return limelight().getEntry("tid").getDouble(-1);
    }
    public double getTargetArea(){
        return limelight().getEntry("ta").getDouble(0.0);
    }
}