package org.firstinspires.ftc.teamcode.Iteration4;

import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import FTCEngine.Core.OpModeBase;

public class SequenceA extends CommonSequence
{
	public SequenceA(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	protected void queueJobs()
	{
		Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
		WobbleGrabber wobbleGrabber = opMode.getBehavior(WobbleGrabber.class);
		Launcher launcher = opMode.getBehavior(Launcher.class);
		Intake intake = opMode.getBehavior(Intake.class);
	}
}
