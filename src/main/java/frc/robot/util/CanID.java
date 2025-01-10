package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;

public class CanID {
  private final String busName;
  private final int deviceId;

  public CanID(int deviceId, String busName) {
    this.deviceId = deviceId;
    this.busName = busName;
  }

  public CanID(int deviceId) {
    this(deviceId, "");
  }

  public String busName() {
    return busName;
  }

  public int deviceId() {
    return deviceId;
  }

  public TalonFX getTalon() {
    return new TalonFX(deviceId, busName);
  }
}
