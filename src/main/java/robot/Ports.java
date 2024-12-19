package robot;

public final class Ports {
  public static final class OI {
    public static final int OPERATOR = 0;
    public static final int DRIVER = 1;
  }

  public static final class Drive {
    public static final int RIGHT_LEADER = 2;
    public static final int RIGHT_FOLLOWER = 3;
    public static final int LEFT_LEADER = -2;
    public static final int LEFT_FOLLOWER = -3;
    // etc

    public static final int GYRO_CHANNEL = 1;

  }
}
