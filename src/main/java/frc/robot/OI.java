/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DoFileTrajectory;
import frc.robot.commands.DoTeleopRobotOrient;
import frc.robot.commands.DoTeleopPosition;
import frc.robot.commands.Motion.ResetEncoders;
import frc.robot.commands.Motion.ResetGyro;
import frc.robot.commands.DeleteAllPrefs;
import frc.robot.commands.Trajectories.*;
import frc.robot.commands.Motion.LogDriveData;
import frc.robot.commands.Elevator.LogElevatorData;
import frc.robot.commands.Limelight.*;
import frc.robot.LimelightControlMode.*;
import frc.robot.LimeLight;
import frc.robot.Gamepad;
import frc.robot.Constants;
import frc.robot.commands.Elevator.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public JoystickButton elevatorToLowerHatch;
    public JoystickButton elevatorToMidRocket;
    public JoystickButton elevatorToTopRocket;

    public JoystickButton jogElevator;

    public Joystick driverController = new Joystick(RobotMap.OI_DRIVER_CONTROLLER);
    public Gamepad gamepad = new Gamepad(RobotMap.OI_CO_DRIVER_CONTROLLER);

    public OI() {
        Timer.delay(.02);

        SmartDashboard.putData(new ResetEncoders());
        Timer.delay(.02);
        SmartDashboard.putData(new ResetGyro());
        Timer.delay(.02);
        SmartDashboard.putData("OrientRobot", new DoTeleopRobotOrient());
        Timer.delay(.02);
        SmartDashboard.putData("DriveToTarget", new DoTeleopPosition());
        Timer.delay(.02);
        SmartDashboard.putData("File Traj", new DoFileTrajectory());
        Timer.delay(.02);
        SmartDashboard.putData("LEDS On", new SetLimelightLeds(LedMode.kforceOn));
        SmartDashboard.putData("LEDS Off", new SetLimelightLeds(LedMode.kforceOff));

        SmartDashboard.putData("Toggle View", new ToggleCamMode());
        SmartDashboard.putData("Toggle Stream", new ToggleStreamMode());

        SmartDashboard.putData("BuffToAct",new BufferToActiveTrajectory(2));

        SmartDashboard.putData("Log Drive", new LogDriveData("Drive", "Drive", 10));

        
        SmartDashboard.putData("Log Elevator", new LogElevatorData( 10));

        SmartDashboard.putData("BuffToAct", new BufferToActiveTrajectory(0));
        /**
         * Co driver controller
         * 
         */

        elevatorToLowerHatch = gamepad.getButtonB();

        elevatorToLowerHatch.whenPressed(new SetElevatorTargetHeight(Constants.ELEVATOR_LOWER_HATCH_INCHES));

        elevatorToMidRocket = gamepad.getButtonY();

        elevatorToMidRocket.whenPressed(new SetElevatorTargetHeight(Constants.ELEVATOR_MID_ROCKET_INCHES));

        elevatorToTopRocket = gamepad.getButtonX();

        elevatorToTopRocket.whenPressed(new SetElevatorTargetHeight(Constants.ELEVATOR_TOP_ROCKET_INCHES));

        jogElevator = gamepad.getStartButton();
        jogElevator.whileHeld(new RunElevatorFromGamepad());

    }
}
