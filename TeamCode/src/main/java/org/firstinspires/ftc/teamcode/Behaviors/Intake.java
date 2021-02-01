package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Auto.AutoBehavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;

public class Intake extends AutoBehavior<Intake.Job>
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public Intake(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		intake = hardwareMap.dcMotor.get("intake");
		apply();
	}

	private DcMotor intake;
	private float power;

	@Override
	public void update()
	{
		super.update();

		if (!opMode.hasSequence())
		{
			float input = opMode.input.getVector(Input.Source.CONTROLLER_2, Input.Button.RIGHT_JOYSTICK).y;
			power = Mathf.normalize(input) * (float)Math.pow(Math.abs(input), 0.75f);
		}

		apply();
	}

	private void apply()
	{
		intake.setPower(power);
	}

	@Override
	protected void updateJob()
	{
		Intake.Job job = getCurrentJob();

		if (job instanceof Run)
		{
			Intake.Run run = (Intake.Run)job;

			intake.setPower(run.power);
			run.finishJob();
		}
	}

	abstract static class Job extends FTCEngine.Core.Auto.Job
	{
	}

	public static class Run extends Intake.Job
	{
		public Run(float power)
		{
			this.power = power;
		}

		public final float power;
	}
}
