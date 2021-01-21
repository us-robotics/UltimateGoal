package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Behaviors.Launcher;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;

public class LauncherAuto extends AutoBehavior<LauncherAuto.Job>
{
	public LauncherAuto(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		launcher = opMode.getBehavior(Launcher.class);
	}

	private Launcher launcher;

	@Override
	protected void updateJob()
	{
		LauncherAuto.Job job = getCurrentJob();

		if (job instanceof Prime)
		{
			Prime prime = (Prime)job;

			launcher.setPrimed(prime.primed);
			prime.finishJob();
		}

		if (job instanceof Hit)
		{
			Hit hit = (Hit)job;

			launcher.setHit(hit.hit);
			hit.finishJob();
		}
	}

	static class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Prime extends Job
	{
		public Prime(boolean primed)
		{
			this.primed = primed;
		}

		public final boolean primed;
	}

	public static class Hit extends Job
	{
		public Hit(boolean hit)
		{
			this.hit = hit;
		}

		public final boolean hit;
	}
}
