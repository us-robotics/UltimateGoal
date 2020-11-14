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

				drivetrain.setDirectInputs(job.positionalInput, job.rotationalInput);
				job.finishJob();

				break;
		}
	}

	static class Job extends FTCEngine.Core.Auto.Job
	{
		public Job(Vector2 positionalInput, float rotationalInput)
		{
			type = JobType.DIRECT_DRIVE;

			this.positionalInput = positionalInput;
			this.rotationalInput = rotationalInput;
		}

		public final JobType type;

		public final Vector2 positionalInput;
		public final float rotationalInput;
	}

	enum JobType
	{
		DIRECT_DRIVE;
	}
}
