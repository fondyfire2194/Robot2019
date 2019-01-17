package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.Auto.CLHabStart.*;
import frc.robot.commands.Auto.*;

public enum PathSelectAuto {
	/*
	 * Order of parameters is Trajectory File, trajectory Gains, stop side and stop
	 * segmnts for switch only, first move command,second trajectory command, second
	 * move command
	 * 
	 */

	CHAB1LC(new CHab1ToLC(), new CHab1ToLC(), new CHab1ToLC(), new CHab1ToLC()), //
	CHAB1RC(new CHab1ToRC(), new CHab1ToRC(), new CHab1ToLC(), new CHab1ToLC()), //

	LHAB1ToCS2(new CHab1ToLC(), new CHab1ToLC(), new CHab1ToLC(), new CHab1ToLC()), //
	RHAB1ToCS2(new CHab1ToLC(), new CHab1ToLC(), new CHab1ToLC(), new CHab1ToLC());

	private Command first;
	private Command second;
	private Command third;
	private Command fourth;

	/*
	 * 4 oommand constructor constructor
	 */
	PathSelectAuto(Command first, Command second, Command third, Command fourth) {

		this.first = first;
		this.second = second;
		this.third = third;
		this.fourth = fourth;

	}

	/*
	 * 
	 * 
	 */

	void build() {

		Robot.autonomousCommand[1] = this.first;

		Robot.autonomousCommand[2] = this.second;

		Robot.autonomousCommand[3] = this.third;

		Robot.autonomousCommand[4] = this.fourth;

	}
}
