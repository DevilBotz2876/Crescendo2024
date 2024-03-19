package frc.robot.subsystems.led;

public interface Led {
    public default void setColor(int red, int blue, int green) {}
    public default int getRed() {return 0;}
    public default int getGreen() {return 0;}
    public default int getBlue() {return 0;}
    
}
