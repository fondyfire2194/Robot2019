/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PathfinderReverseTrajectoryUsingNotifier;
import frc.robot.commands.PathfinderTrajectoryUsingNotifier;
import frc.robot.commands.RobotOrient;
import frc.robot.commands.RobotPosition;
import frc.robot.BuildTrajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RobotRotate;
import jaci.pathfinder.Trajectory;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  public static DriveTrain driveTrain = null;
  public static RobotRotate robotRotate;
  public static SimpleCSVLogger simpleCSVLogger;
  public static GeneratePositionTrajectory generatePositionTrajectory;
  public static String chosenFileName = "Dummy";
  public static OI m_oi;
  public static Preferences prefs;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static boolean doRobotPosition;
  public static double positionTargetFt;
  public static boolean incrPosn;
  public static boolean isPositioning;
  public static boolean stopPositioning;
  public static boolean orientRunning;
  public static boolean reverseOrient;
  public static boolean doTeleopOrient;

  private boolean revPosn = false;
  public static double targetPosition;
  public static double targetRate;
  public static double xPosition;
  public static double yPosition;

  private double angleTarget = 90;
  private double orientRate = 0.5;

  public enum motionType {
    incremental, absolute
  }

  public static Trajectory[] activeTrajectory;// = new Trajectory[2];
  public static double[] activeTrajectoryGains = { 0, 0, 0, 0 };
  public static double[] activeTrajectoryTwoGains = { 0, 0, 0, 0 };
  public static boolean doTeleopTrajectory;
  public static boolean revTraj;;
  public static boolean doFileTrajectory;
  public static boolean trajectoryRunning;
  public static boolean buildOK;
  private SendableChooser<Integer> testTrajectoryChooser;
  private int testTrajectory;

  public static String[] names = { "Step", "LeftCmd", "LeftFt", "RightCmd", "RightFt", "AngleCmd", "Angle",
      "LeftSegVel", "left", "ActLeftVel", "RightSegVel", "right", "ActRightVel", "turn" };
  public static String[] units = { "Number", "FT", "FT", "FT", "FT", "Deg", "Deg", "pct", "pct", "pct", "pct", "pct",
      "pct", "pct" };
  public static boolean createTrajectoryRunFile = true;
  public static double trajectoryX;
  public static double trajectoryY;
  public static double trajectoryAngle;
  public static String trajFileName;
  public static String logName;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
    robotRotate = new RobotRotate();
    m_oi = new OI();
    simpleCSVLogger = new SimpleCSVLogger();
    generatePositionTrajectory = new GeneratePositionTrajectory();
    prefs = Preferences.getInstance();
      //  Pref.deleteUnused();
    Pref.addMissing();
    SmartDashboard.putData(driveTrain);
    Timer.delay(.02);
    SmartDashboard.putNumber("Posn_Target", 4);
    Timer.delay(.02);
    SmartDashboard.putNumber("PosnTargetRate", .5);
    Timer.delay(.02);
    positionTargetFt = Robot.driveTrain.getLeftEncoderCount() / Constants.DRIVE_ENCODER_CTS_PER_FT;
    SmartDashboard.putNumber("Target Angle", angleTarget);
    Timer.delay(.02);
    SmartDashboard.putNumber("Orient Rate", orientRate);
    Timer.delay(.02);
    SmartDashboard.putBoolean("RevOrient", false);
    Timer.delay(.02);
    SmartDashboard.putBoolean("Incremental", incrPosn);
    Timer.delay(.02);
    SmartDashboard.putBoolean("RevPosn", revPosn);
    Timer.delay(.02);
    // trajectory entries
    SmartDashboard.putNumber("TrajX", 10);
    Timer.delay(.02);
    SmartDashboard.putNumber("TrajY", 0);
    Timer.delay(.02);
    SmartDashboard.putNumber("TrajAngle", 0);
    Timer.delay(.02);
    SmartDashboard.putBoolean("RevTraj", false);

    testTrajectoryChooser = new SendableChooser<Integer>();

    testTrajectoryChooser.addDefault("TestTrajA", 0);
    testTrajectoryChooser.addObject("TestTrajB", 1);
    testTrajectoryChooser.addObject("TestTrajC", 2);
    testTrajectoryChooser.addObject("TestTrajD", 3);
    testTrajectoryChooser.addObject("TestTrajE", 4);
    testTrajectoryChooser.addObject("TestTrajF", 5);

    SmartDashboard.putData("Trajectory Chooser", testTrajectoryChooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    updateStatus();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    if (doRobotPosition) {
      targetPosition = SmartDashboard.getNumber("Posn_Target", 5);
      targetRate = SmartDashboard.getNumber("PosnTargetRate", .5);
      if (SmartDashboard.getBoolean("RevPosn", false))
        targetPosition = -targetPosition;
      if (SmartDashboard.getBoolean("Incremental", false)) {
        new RobotPosition(targetPosition, targetRate, motionType.absolute, stopPositioning, 20.).start();
      } else
        new RobotPosition(targetPosition, targetRate, motionType.incremental, stopPositioning, 20.).start();

      doRobotPosition = false;
    }
    if (doTeleopOrient) {
      // sensors.resetGyro();

      angleTarget = SmartDashboard.getNumber("Target Angle", 90);
      if (SmartDashboard.getBoolean("ReverseOrient", false))
        angleTarget = -angleTarget;
      orientRate = SmartDashboard.getNumber("Orient Rate", .25);
      new RobotOrient(angleTarget, orientRate, true, 30).start();
      doTeleopOrient = false;
    }

    /**
     * Create trajectory from Shuffleboard x, y and angle fields put it in active
     * trajectory array. [0] is left [1] is right get the 4 trajectory loop gains
     * (P, D, A and Turn)from prefs start the trajectory notifier and data logger
     * 
     * 
     */
    if (doTeleopTrajectory) {
      trajectoryX = SmartDashboard.getNumber("TrajX", 10);
      trajectoryY = SmartDashboard.getNumber("TrajY", 0);
      trajectoryAngle = SmartDashboard.getNumber("TrajAngle", 0);
 activeTrajectory = 
     generatePositionTrajectory.generate(trajectoryX, trajectoryY, trajectoryAngle);
     logName = "Manual";
    }
    if (doFileTrajectory) {

      testTrajectory = testTrajectoryChooser.getSelected();

      switch (testTrajectory) {

      case 0:
        trajFileName = "TestTrajA";
        break;
      case 1:
        trajFileName = "TestTrajB";
        break;
      case 2:
        trajFileName = "TestTrajC";
        break;
      case 3:
        trajFileName = "TestTrajD";
        break;
      case 4:
        trajFileName = "TestTrajE";
        break;
      case 5:
        trajFileName = "TestTrajF";
        break;
      default:
        trajFileName = "None";
        break;
      }
 logName = trajFileName;
      activeTrajectory = BuildTrajectory.buildFileName(true, trajFileName);
      SmartDashboard.putBoolean("FileOK", buildOK);
      
      if (!buildOK) {
        doFileTrajectory = false;
        DriverStation.reportError("Error reading file", true);
      }
    }
    if (doTeleopTrajectory || (doFileTrajectory && buildOK)) {

      driveTrain.resetEncoders();
      driveTrain.resetGyro();
      xPosition = activeTrajectory[0].get(0).x;
      yPosition = activeTrajectory[0].get(0).y - (Constants.WHEELBASE_WIDTH / 2);
      constantsFromPrefs();

      if (!SmartDashboard.getBoolean("RevTraj", false)) {
        new PathfinderTrajectoryUsingNotifier().start();
        constantsFromPrefs();
      } else {
        new PathfinderReverseTrajectoryUsingNotifier().start();
        revConstantsFromPrefs();
      }
      trajectoryRunning = true;
      doTeleopTrajectory = false;
      doFileTrajectory = false;
    }
    doFileTrajectory = false;

    updateStatus();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }

  public void updateStatus() {
    driveTrain.updateStatus();
    SmartDashboard.putBoolean("PosnRng", isPositioning);
    SmartDashboard.putBoolean("TrajRng", trajectoryRunning);
    SmartDashboard.putBoolean("OrientRng", orientRunning);
     SmartDashboard.putNumber("TrajLen", activeTrajectory == null ? 0 : activeTrajectory[0].length());
    SmartDashboard.putNumber("Preftest", Pref.getPref("y"));
    SmartDashboard.putString("FileChosen",chosenFileName);

  }

  private void constantsFromPrefs() {
    activeTrajectoryGains[0] = Pref.getPref("PathKp");// prefs.getDouble("PathP", 0);
    activeTrajectoryGains[1] = Pref.getPref("PathKd");// prefs.getDouble("PathD", 0);
    activeTrajectoryGains[2] = Pref.getPref("PathKa");// prefs.getDouble("PathA", 0);
    activeTrajectoryGains[3] = Pref.getPref("PathKt");// prefs.getDouble("PathTurn", 0);
  }

  private void revConstantsFromPrefs() {
    activeTrajectoryGains[0] = Pref.getPref("PathKpRev");// prefs.getDouble("PathP", 0);
    activeTrajectoryGains[1] = Pref.getPref("PathKdRev");// prefs.getDouble("PathD", 0);
    activeTrajectoryGains[2] = Pref.getPref("PathKaRev");// prefs.getDouble("PathA", 0);
    activeTrajectoryGains[3] = Pref.getPref("PathKtRev");// prefs.getDouble("PathTurn", 0);
  }
}
