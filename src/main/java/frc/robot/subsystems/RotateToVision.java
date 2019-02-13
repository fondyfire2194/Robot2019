/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SD;
import frc.robot.AutoChoosers;

/**
 * Add your docs here.
 */
public class RotateToVision extends PIDSubsystem {
	/**
	 * Add your docs here.
	 */
	private static final double Kp = .015;
	private static final double Ki = 0.0;
	private static final double Kd = 0.05;
	private static final double Kf = 0;

	public RotateToVision() {
		// Intert a subsystem name and PID values here
		super("RotateToVision", Kp, Ki, Kd, Kf);
		getPIDController().setInputRange(-180, 180);
		getPIDController().setOutputRange(-1, 1);
		getPIDController().setContinuous();
		getPIDController().disable();
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	protected double returnPIDInput() {
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
		return Robot.limelightCamera.getdegRotationToTarget();
	}

	@Override
	protected void usePIDOutput(double output) {
		Robot.driveTrain.leftDriveOut(output);
		Robot.driveTrain.rightDriveOut(-output);
	}

	public void enablePID() {
		getPIDController().enable();
	}

	public void disablePID() {
		getPIDController().disable();
	}

	@Override
	public void setSetpoint(double setpoint) {
		getPIDController().setSetpoint(setpoint);
	}

	public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
		getPIDController().setPID(Kp, Ki, Kd, Kf);
	}

	public void setMaxOut(double speed) {
		getPIDController().setOutputRange(-speed, speed);
	}

	public double getKp() {
		return getPIDController().getP();
	}

	public double getError() {
		return getPIDController().getError();
	}

	public boolean inPosition() {
		return (Math.abs(getError()) <= 5);
	}

	public boolean closeToPosition() {
		return (Math.abs(getError()) <= 8);
	}

	public boolean isEnabled() {
		return getPIDController().isEnabled();
	}

	public void updateStatus() {

		if (AutoChoosers.debugChooser.getSelected() == 7) {
			SmartDashboard.putBoolean("VisRotateInPos", inPosition());
			SD.putN1("VISSetpoint", getSetpoint());
			SmartDashboard.putBoolean("Rotate Enabled?", isEnabled());
			SD.putN1("VisOrient Error", getError());

		}

	}
}
