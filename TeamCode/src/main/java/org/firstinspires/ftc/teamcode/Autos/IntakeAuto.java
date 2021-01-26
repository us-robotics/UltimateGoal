package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Behaviors.Intake;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;

public class IntakeAuto extends AutoBehavior<IntakeAuto.Job>
{
	public IntakeAuto(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		intake = opMode.getBehavior(Intake.class);
	}

	private Intake intake;

	@Override
	protected void updateJob()
	{
		IntakeAuto.Job job = getCurrentJob();

		if (job instanceof Run)
		{
			IntakeAuto.Run run = (IntakeAuto.Run)job;

			intake.setPower(run.power);
			run.finishJob();
		}
	}

	static class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Run extends IntakeAuto.Job
	{
		public Run(float power)
		{
			this.power = power;
		}

		public final float power;
	}
}
