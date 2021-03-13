//////////////////
/* USE THIS ONE */
//////////////////

package frc.robot;

//General Robot Imports

//import for the motor controllers
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//imports for the control and timers
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

//imports for actually moving the damn thing
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.Sendable;

public class Robot extends TimedRobot {
  // needed objects for drive and time
  private final Joystick stick1 = new Joystick(0);
  private final Joystick stick2 = new Joystick(1);

  /*
   * motor controller objects for differential drive Left Motors
   */

  // depending on motor controller avaliablility, switch between Talon and Victor
  // classes
  WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(5); // mc 1
  WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(6); // mc 2

  SpeedControllerGroup leftGroup = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

  /* Right Motors */

  WPI_VictorSPX m_frontRight = new WPI_VictorSPX(7); // victor right side top
  WPI_VictorSPX m_rearRight = new WPI_VictorSPX(8); // victor right side bottom

  SpeedControllerGroup rightGroup = new SpeedControllerGroup(m_frontRight, m_rearRight);

  // Diff drive = tank drive object
  DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  @Override
  public void robotInit() {
    // Places a compass indicator for the gyro heading on the dashboard
    Shuffleboard.getTab("Robot Systems").add((Sendable) m_drive);

  }

  /**
   * This function is called periodically during teleoperated mode.
   */

  @Override
  public void teleopPeriodic() {
    double throttle = 0.7 * (-stick1.getThrottle() + 1) / 2 + 0.3;
    m_drive.arcadeDrive((-stick1.getY() * throttle) * 0.7 + (-stick2.getY() * throttle) * 0.3,
        (stick1.getX() * throttle) * 0.7 + (stick2.getX() * throttle) * 0.3);
  }
}