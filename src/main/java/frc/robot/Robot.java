//////////////////
/* USE THIS ONE */
//////////////////

package frc.robot;

import java.net.ServerSocket;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//General Robot Imports

  //import for the motor controllers
  import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
  import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
  import edu.wpi.first.wpilibj.VictorSP;

  //imports for the control and timers
  import edu.wpi.first.wpilibj.Joystick;
  import edu.wpi.first.wpilibj.TimedRobot;
  import edu.wpi.first.wpilibj.Timer;

  //imports for actually moving the damn thing
  import edu.wpi.first.wpilibj.drive.DifferentialDrive;
  import edu.wpi.first.wpilibj.SpeedControllerGroup;
  import edu.wpi.first.wpilibj.controller.PIDController;    

  //sensors and control systems
  import edu.wpi.first.wpilibj.SPI;
  import edu.wpi.first.wpilibj.I2C;
  import com.kauailabs.navx.frc.AHRS;
  import edu.wpi.first.wpilibj.DriverStation;

  //colour stuff
  import edu.wpi.first.wpilibj.util.Color;
  import com.revrobotics.ColorSensorV3;
  import com.revrobotics.ColorMatchResult;
  import com.revrobotics.ColorMatch;

  //making things look pretty
  import edu.wpi.first.wpilibj.shuffleboard.*;
  import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.cameraserver.CameraServer;
  import edu.wpi.cscore.UsbCamera;
  // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

 /* Shooter - button toggle full speed
  Intake lift - button toggle speed tbd
  Control Panel Motor - button hold
  Winch Motor - 2nd joystick hold analogue
  2 Intake motors - joined with intake lift motor

  Pneumatics:
  Control Panel thing: toggle


*/

public class Robot extends TimedRobot {

  //config I2C
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  
  //needed objects for drive and time
  private final Joystick stick1 = new Joystick(0);
  private final Joystick stick2 = new Joystick(1);

  Servo feedServo = new Servo(3);

  private final Timer m_timer = new Timer();
  PIDController pid = new PIDController(kP, kI, kD);
  AHRS ahrs;

  String titleColor = "Color";

  //colour stuff
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  String colorString;

  //Buttons
    final static short motorInverseButton = 1;  
    final static short pnButton = 2;
    final static short shooterButton = 3;
    final static short intakeSystemButton = 4;
    final static short winchButton = 5;
    final static short controlPanelButton = 6;

  //toggle control
    Boolean shooterControlOn = false;
    Boolean motorInverseOn = false;
    Boolean intakeControlOn = false; 
    Boolean controlPanelControlOn = false; 

    Boolean motorInversePressed = false;

    Boolean motorInverse = false;
    Boolean shooterControl = false;
    Boolean intakeControl = false;
    Boolean controlPanelControl = false;

  //PID constants
    final static double kP = 1;
    final static double kI = 0.01;
    final static double kD = 0;
  
  /*motor controller objects for differential drive 
    Left Motors */

    //depending on motor controller avaliablility, switch between Talon and Victor classes
    WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(5); //mc 1 
    WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(6);  //mc 2

    SpeedControllerGroup leftGroup = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

  /* Right Motors */

    WPI_VictorSPX  m_frontRight = new WPI_VictorSPX(7); //victor right side top
    WPI_VictorSPX  m_rearRight = new WPI_VictorSPX(8); //victor right side bottom

    SpeedControllerGroup rightGroup = new SpeedControllerGroup(m_frontRight, m_rearRight);
  
  // Diff drive = tank drive object
  DifferentialDrive m_drive = new DifferentialDrive(leftGroup, rightGroup);

  // Secondary Motors

  TalonSRX m_shooter = new TalonSRX(19);

  TalonSRX m_intakeLift = new TalonSRX(13);
  VictorSP m_intakeL = new VictorSP(0);
  VictorSP m_intakeR = new VictorSP(1);
  
  SpeedControllerGroup m_intake = new SpeedControllerGroup(m_intakeL, m_intakeR);

  WPI_VictorSPX m_winchMotor = new WPI_VictorSPX(13);

  WPI_VictorSPX m_controlPanelSpin = new WPI_VictorSPX(14);

  final static int gyroPort = 1;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  @Override
  public void robotInit() {

    //colour config
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget); 
    
    m_intakeR.setInverted(true);

    /*try {

      ahrs = new AHRS(SPI.Port.kMXP);

    } catch (RuntimeException ex) {

      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage() + "sorry just ask Ryan lmao", true);

    }*/

    //livestream camera
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(640, 480);

    // Places a compass indicator for the gyro heading on the dashboard
    //Shuffleboard.getTab("Robot Systems").add((Sendable) ahrs );
    Shuffleboard.getTab("Robot Systems").add(camera);
    Shuffleboard.getTab("Robot Systems").add((Sendable) m_drive);
    Shuffleboard.getTab("Robot Systems").add((Sendable) m_intake);
    Shuffleboard.getTab("Robot Systems").add((Sendable) pid);
    // Shuffleboard.getTab("Robot Systems").add(titleColor, colorString);


  }

  @Override
  public void robotPeriodic() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

  }

  /**
  * This function is run once each time the robot enters autonomous mode.
  */
  @Override
  public void autonomousInit() {

    m_timer.reset();
    m_timer.start();

    pid.reset();

  }

  /**
  * This function is called periodically during autonomous.
  */

  /*
  @Override
  public void autonomousPeriodic() {

    final float setpoint = ahrs.getCompassHeading(); //set desired compass heading as start heading
    // if(ahrs.getCompassHeading() >= 270) 
      final double minimumInput = ahrs.getCompassHeading() - 90;
    
    
    final double maximumInput = ahrs.getCompassHeading() + 90;
    
    pid.enableContinuousInput(minimumInput, maximumInput);

    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_drive.arcadeDrive(0.5, pid.calculate(ahrs.getCompassHeading(), setpoint)); //move at half speed, aiming for set heading
    } else {
      m_drive.stopMotor(); // stop robot
    }

  }*/

  /**
  * This function is called once each time the robot enters teleoperated mode.
  */
  @Override
  public void teleopInit() {

    pid.reset();

  }

  /**
  * This function is called periodically during teleoperated mode.
  */

  @Override
  public void teleopPeriodic() {

    // motorInverse.updateToggle(stick1, motorInverseButton, motorInverseOn, motorInversePressed);

    // //drive controls
      double motorSpeed = stick1.getRawAxis(3);  // Get the raw value
      motorSpeed = motorSpeed + 1;                                 // Range of 0-2
      motorSpeed = motorSpeed / 2;                                 // Range of 0-1

      double throttle = 0.7 * (-stick1.getThrottle() + 1) / 2 + 0.3;
      m_drive.arcadeDrive(-stick1.getY() * throttle, stick1.getX() * throttle);

    
    // shooter toggle
      if(stick2.getRawButtonPressed(shooterButton)) {
        shooterControl = !shooterControl;
      }
    
     if(shooterControl) {
       double shooterThrottle = 0.7 * (-stick2.getThrottle() + 1) / 2 + 0.3;
       m_shooter.set(ControlMode.PercentOutput, -shooterThrottle); //between -1 and 1 speed
     } else {
       m_shooter.set(ControlMode.PercentOutput, 0);
     }
    
    // // intake toggle
     if(stick2.getRawButtonPressed(intakeSystemButton)) 
       intakeControl = !intakeControl;

     if(intakeControl) {

       m_intakeLift.set(ControlMode.PercentOutput, 0.8);
       m_intake.set(1);
    
     } else {
      m_intakeLift.set(ControlMode.PercentOutput, 0);
      m_intake.set(0);
     }

     if (stick2.getRawButton(1)) {
       feedServo.set(1);
     } else {
       feedServo.set(0.5);
     }
    

    // // winch hold
    // m_winchMotor.set(ControlMode.PercentOutput, stick2.getY());

    // // control panel spin hold
    // if(stick2.getRawButtonPressed(controlPanelButton)) 
    //   m_controlPanelSpin.set(ControlMode.PercentOutput, 0.1);    
  
  }

  /*
  * This function is called periodically during test mode.
  */
  @Override
  public void testPeriodic() {

  }
}