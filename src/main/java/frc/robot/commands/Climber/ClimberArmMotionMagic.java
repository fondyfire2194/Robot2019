package frc.robot.commands.Climber;

import frc.robot.Robot;
import frc.robot.Pref;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimberArmMotionMagic extends Command {
	

	public ClimberArmMotionMagic() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.climberArm);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.climberArm.climberArm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		Robot.climberArm.climberArm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		Robot.climberArm.climberArm.selectProfileSlot(0, 0);

		Robot.climberArm.climberArm.configOpenloopRamp(0, 0);
		Robot.climberArm.climberArm.configClosedloopRamp(0, 0);
		Robot.climberArm.climberArm.configPeakOutputForward(1, 0);
		Robot.climberArm.climberArm.configPeakOutputReverse(-1, 0);
		Robot.climberArm.climberArm.configNominalOutputForward(0, 0);
		Robot.climberArm.climberArm.configNominalOutputReverse(0, 0);
		Robot.climberArm.climberArm.selectProfileSlot(0, 0);

		Robot.climberArm.climberArm.config_kF(0, Pref.getPref("ArmMMKf"), 0);
		Robot.climberArm.climberArm.config_kP(0, Pref.getPref("ArmMMKp"), 0);
		Robot.climberArm.climberArm.config_kI(0, Pref.getPref("ArmMMKi"), 0);
		Robot.climberArm.climberArm.config_kD(0, Pref.getPref("ArmMMKd"), 0);

		Robot.climberArm.armTargetDegrees = Robot.climberArm.getArmDegrees();
		Robot.climberArm.lastHoldDegrees = Robot.climberArm.armTargetDegrees + .01;

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.climberArm.armTargetDegrees != Robot.climberArm.lastHoldDegrees) {
			Robot.climberArm.armMagicMotion(Robot.climberArm.armTargetDegrees, Constants.CLIMBER_ARM_RATE);
			Robot.climberArm.lastHoldDegrees = Robot.climberArm.armTargetDegrees;
;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
