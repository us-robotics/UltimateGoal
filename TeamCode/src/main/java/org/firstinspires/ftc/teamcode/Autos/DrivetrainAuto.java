package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Behaviors.MecanumDrivetrain;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public class DrivetrainAuto extends AutoBehavior<DrivetrainAuto.Job>
{
	public DrivetrainAuto(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);
		drivetrain = opMode.getBehavior(MecanumDrivetrain.class);
	}

	//following Gary's syntax and declaring down here to separate methods
	private MecanumDrivetrain drivetrain;

	@Override
	protected void updateJob()
	{
		Job job = getCurrentJob();

		switch (job.type)
		{
			case DIRECT_DRIVE:

				System.out.println(job.positionalMovement);

				drivetrain.setMovements(job.positionalMovement, job.rotationalMovement);
				job.finishJob();

				break;
		}
	}

	static class Job extends FTCEngine.Core.Auto.Job
	{
		public Job(Vector2 positionalMovement, float rotationalMovement)
		{
			type = JobType.DIRECT_DRIVE;

			this.positionalMovement = positionalMovement;
			this.rotationalMovement = rotationalMovement;
		}

		public final JobType type;

		public final Vector2 positionalMovement;
		public final float rotationalMovement;
	}

	enum JobType
	{
		DIRECT_DRIVE;
	}
}
