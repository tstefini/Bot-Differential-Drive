package robot;

import static edu.wpi.first.units.Units.Seconds;
import static robot.Constants.PERIOD;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.CommandRobot;
import lib.FaultLogger;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.urcl.URCL;
import robot.Ports.OI;

public class Robot extends CommandRobot implements Logged {

  // INPUT DEVICES
  private final CommandXboxController operator = new CommandXboxController(OI.OPERATOR);
  private final CommandXboxController driver = new CommandXboxController(OI.DRIVER);

  // POWER DISTRIBUTION HUB
  private final PowerDistribution pdh = new PowerDistribution();

  // SUBSYSTEMS
  private Drive drive = new Drive();

  // COMMANDS
  public Robot() {
    super(PERIOD.in(Seconds));
    configureGameBehavior();
    configureBindings();
  }

  // Configures basic behavior for different periods during the game
  private void configureGameBehavior() {
    DataLogManager.start();
    Monologue.setupMonologue(this, "/Robot", false, true);
    addPeriodic(Monologue::updateAll, PERIOD.in(Seconds));
    addPeriodic(FaultLogger::update, 2);

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("PDH", pdh);
    FaultLogger.register(pdh);

    RobotController.setBrownoutVoltage(6.0);

    if (isReal()) {
      URCL.start();
      pdh.clearStickyFaults();
      pdh.setSwitchableChannel(true);
    } else {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  // Configures trigger -> command bindings
  private void configureBindings() {
    drive.setDefaultCommand(drive.drive(driver::getLeftY, driver::getRightY));
  }

  // Command factory to make both controllers rumble
  public Command rumble(RumbleType rumbleType, double strength) {
    return Commands.runOnce(
            () -> {
              driver.getHID().setRumble(rumbleType, strength);
              operator.getHID().setRumble(rumbleType, strength);
            })
        .andThen(Commands.waitSeconds(0.3))
        .finallyDo(
            () -> {
              driver.getHID().setRumble(rumbleType, 0);
              operator.getHID().setRumble(rumbleType, 0);
            });
  }

  // Periodic function to drive the robot
  public Command drive(DoubleSupplier vLeft, DoubleSupplier vRight) {
    return Commands.run(() -> {
      // You'd call your hardware control here to set motor speeds
      // Example: leftMotor.set(vLeft.getAsDouble());
      // Example: rightMotor.set(vRight.getAsDouble());
      System.out.println("Driving left: " + vLeft.getAsDouble() + " right: " + vRight.getAsDouble());
    });
  }

  // Cleanup and close resources
  @Override
  public void close() {
    super.close();
  }

  // Simple Drive subsystem (assuming it's missing or needs to be implemented)
  public static class Drive {
    public Command drive(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
      return Commands.run(
        () -> {
          // Here you'd call your robot hardware APIs to control motors
          // For example:
          // leftMotor.set(leftSpeed.getAsDouble());
          // rightMotor.set(rightSpeed.getAsDouble());
          System.out.println("Drive command - left: " + leftSpeed.getAsDouble() + " right: " + rightSpeed.getAsDouble());
        });
    }

    public void setDefaultCommand(Command command) {
      // If using a default command, configure it here, for example:
      // defaultCommand = command;
    }
  }
}
