package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.Auto.CR1HabStart.*;
import frc.robot.commands.Auto.CL1HabStart.*;
// import frc.robot.commands.Auto.LHab1Start.*;

import frc.robot.commands.Auto.*;

public enum PathSelectAuto {
	/*
	 * Order of parameters is Trajectory File, trajectory Gains, stop side and stop
	 * segmnts for switch only, first move command,second trajectory command, second
	 * move command
	 * 
	 */

	CHAB1LC(new CHab1ToLC(), new LCToLoadApproach(), new AdjustLoadApproach(), new MoveToAndLoadPanel(),
	new LoadToCS2()), //

	CHAB1RC(new CHab1ToRC(), new CHab1ToRC(), new CHab1ToLC(), new CHab1ToLC()), //

	LHAB1ToCS2(new CHab1ToLC(), new CHab1ToLC(), new CHab1ToLC(), new CHab1ToLC()), //

	RHAB1ToCS2(new LoadToCS2(), new CHab1ToLC(), new CHab1ToLC(), new CHab1ToLC());

	private Command first;
	private Command second;
	private Command third;
	private Command fourth;
	private Command fifth;

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
	 * 5 oommand constructor constructor
	 */
	PathSelectAuto(Command first, Command second, Command third, Command fourth, Command fifth) {

		this.first = first;
		this.second = second;
		this.third = third;
		this.fourth = fourth;
		this.fifth = fifth;

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

		Robot.autonomousCommand[5] = this.fifth;

	}
}
