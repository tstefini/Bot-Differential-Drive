package robot.drive;
// has it worked??
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

public class DriveConstants {
  // Example constants; adjust these values to match your robot
  public static final double POSITION_FACTOR = 0.0001; // Example value for encoder to meters
  public static final double VELOCITY_FACTOR = 0.0001; // Example value for encoder velocity to meters per second

  // Example PID constants for a basic control system
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  // Constants for feedforward
  public static final double kS = 1.0; // Static voltage to overcome friction
  public static final double kV = 0.5; // Velocity constant (tune for your system)

  // Drivetrain physical constants
  public static final double GEARING = 10.0; // Gear ratio (e.g., 10:1)
  public static final double MOI = 0.5; // Moment of inertia (kg m^2)
  public static final double DRIVE_MASS = 50.0; // Mass of the robot (kg)
  public static final double WHEEL_RADIUS = 0.076; // Wheel radius (meters)
  public static final double TRACK_WIDTH = 0.5; // Track width (meters)
  public static final Matrix<N7, N1> STD_DEVS = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0);
  public static final double MAX_SPEED = 0;

}
