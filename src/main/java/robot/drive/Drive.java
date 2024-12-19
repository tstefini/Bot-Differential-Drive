package robot.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import robot.Constants;
import robot.Ports;
import robot.Robot;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

public class Drive extends SubsystemBase {
    // Motor controllers and encoders
    private final CANSparkMax leftLeader = new CANSparkMax(Ports.Drive.LEFT_LEADER, MotorType.kBrushless);
    private final CANSparkMax leftFollower = new CANSparkMax(Ports.Drive.LEFT_FOLLOWER, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Ports.Drive.RIGHT_LEADER, MotorType.kBrushless);
    private final CANSparkMax rightFollower = new CANSparkMax(Ports.Drive.RIGHT_FOLLOWER, MotorType.kBrushless);

    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
    private final AnalogGyro gyro = new AnalogGyro(Ports.Drive.GYRO_CHANNEL);

    // Feedforward values
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FF.kS, FF.kV);

    // PID Controllers
    private final PIDController leftPIDController = new PIDController(PID.kP, PID.kI, PID.kD);
    private final PIDController rightPIDController = new PIDController(PID.kP, PID.kI, PID.kD);

    // Odometry object
    private final DifferentialDriveOdometry odometry;

    // Simulation object
    private final DifferentialDrivetrainSim driveSim;

    public Drive() {
        // Initialize motor controllers and encoders
        for (CANSparkMax spark : List.of(leftLeader, leftFollower, rightLeader, rightFollower)) {
            spark.restoreFactoryDefaults();
            spark.setIdleMode(IdleMode.kBrake);
        }
        rightFollower.follow(rightLeader);
        leftFollower.follow(leftLeader);
        leftLeader.setInverted(true);

        // Initialize odometry with an initial pose
        odometry = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0, new Pose2d());

        // Set up encoder conversion factors (adjust according to your system)
        leftEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
        rightEncoder.setPositionConversionFactor(DriveConstants.POSITION_FACTOR);
        leftEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);
        rightEncoder.setVelocityConversionFactor(DriveConstants.VELOCITY_FACTOR);

        // Initialize the gyro
        gyro.reset();

        // Initialize the simulation model
        driveSim = new DifferentialDrivetrainSim(
                DCMotor.getMiniCIM(2),
                DriveConstants.GEARING,
                DriveConstants.MOI,
                DriveConstants.DRIVE_MASS,
                DriveConstants.WHEEL_RADIUS,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.STD_DEVS);
        // ...
    }

    // Method to reset odometry
    public void resetOdometry(Pose2d pose) {
        // Reset odometry using the current encoder positions and gyro angle
        DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(
                leftEncoder.getPosition(),
                rightEncoder.getPosition());
        odometry.resetPosition(new Rotation2d(gyro.getAngle()), wheelPositions, pose);
    }

    // Method to drive the robot using PID control and feedforward
    public void drive(double leftSpeed, double rightSpeed, boolean resetOdometry) {
        // Calculate feedforward values for each motor
        double leftFeedforward = feedforward.calculate(leftSpeed);
        double rightFeedforward = feedforward.calculate(rightSpeed);

        // Use PID controllers to calculate the adjusted speed
        double leftOutput = leftPIDController.calculate(leftEncoder.getVelocity(), leftSpeed) + leftFeedforward;
        double rightOutput = rightPIDController.calculate(rightEncoder.getVelocity(), rightSpeed) + rightFeedforward;

        // Apply the calculated output values to the motors
        leftLeader.set(leftOutput);
        rightLeader.set(rightOutput);

        // If resetOdometry flag is true, reset the odometry
        if (resetOdometry) {
            resetOdometry(new Pose2d(0, 0, new Rotation2d(0))); // Reset to (0, 0) with no rotation
        }

        // Update the odometry with current encoder positions and gyro angle
        odometry.update(new Rotation2d(gyro.getAngle()),
                leftEncoder.getPosition(),
                rightEncoder.getPosition());

        // For simulation, set the inputs for the drivetrain sim
        driveSim.setInputs(leftOutput, rightOutput);
    }

    // Overloaded method without odometry reset
    public void drive(double leftSpeed, double rightSpeed) {
        // Apply speed limits (if any) defined in DriveConstants
        final double realLeftSpeed = leftSpeed * DriveConstants.MAX_SPEED;
        final double realRightSpeed = rightSpeed * DriveConstants.MAX_SPEED;

        // Calculate feedforward values
        final double leftFeedforward = feedforward.calculate(realLeftSpeed);
        final double rightFeedforward = feedforward.calculate(realRightSpeed);

        // Calculate PID output using the encoder's velocity
        final double leftPID = leftPIDController.calculate(leftEncoder.getVelocity(), realLeftSpeed);
        final double rightPID = rightPIDController.calculate(rightEncoder.getVelocity(), realRightSpeed);

        // Calculate final voltage for the motors
        double leftVoltage = leftPID + leftFeedforward;
        double rightVoltage = rightPID + rightFeedforward;

        // Apply the calculated voltage to the motors
        leftLeader.setVoltage(leftVoltage);
        rightLeader.setVoltage(rightVoltage);

        // For simulation, set the inputs for the drivetrain sim
        driveSim.setInputs(leftVoltage, rightVoltage);
    }

    // Periodic method for real robot
    public void periodic() {
        // Update the odometry based on whether we are using a real robot or simulation
        if (Robot.isReal()) {
            // For the real robot, use the actual gyro angle
            odometry.update(gyro.getRotation2d(),
                    leftEncoder.getPosition(),
                    rightEncoder.getPosition());
        } else {
            // For simulation, use the simulated heading and encoder positions
            odometry.update(driveSim.getHeading(),
                    driveSim.getLeftPositionMeters(),
                    driveSim.getRightPositionMeters());
        }

        field2d.setRobotPose(pose());

    }

    private Pose2d pose() {
        throw new UnsupportedOperationException("Unimplemented method 'pose'");
    }

    @Log.NT
    private final Field2d field2d = new Field2d();

    // Simulation periodic method
    @Override
    public void simulationPeriodic() {
        // Update simulation with time step
        driveSim.update(Constants.PERIOD.in(Seconds));
        leftEncoder.setPosition(driveSim.getLeftPositionMeters());
        rightEncoder.setPosition(driveSim.getRightPositionMeters());

    }

    // Constants for feedforward and PID values
    public static final class FF {
        public static final double kS = 1.0; // Static voltage to overcome friction
        public static final double kV = 3.0; // Velocity constant
    }

    public static final class PID {
        public static final double kP = 8.5; // Proportional constant
        public static final double kI = 0.0; // Integral constant (can be adjusted)
        public static final double kD = 0.0; // Derivative constant (can be adjusted)
    }

    // Method to get the current pose from odometry
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Method to get the current gyro angle
    public double getGyroAngle() {
        return gyro.getAngle();
    }

    // Method to get the current wheel positions
    public DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(leftEncoder.getPosition(), rightEncoder.getPosition());
    }

}
