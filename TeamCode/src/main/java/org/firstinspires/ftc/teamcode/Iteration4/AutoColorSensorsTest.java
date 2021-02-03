package org.firstinspires.ftc.teamcode.Iteration4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.CameraVision;
import org.firstinspires.ftc.teamcode.Behaviors.ColorSensors;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Auto.JobSequence;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;

@Autonomous(name = "Ultimate Goal AutoColorSensorsTest")
public class AutoColorSensorsTest extends OpModeBase
{
	@Override
	protected void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new Drivetrain(this));
		behaviorList.add(new InertialMeasurementUnit(this));
		behaviorList.add(new ColorSensors(this));
	}

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{

	}

	@Override
	public void init()
	{
		super.init();
		assignSequence(new Sequence(this));
	}

	private static class Sequence extends JobSequence
	{
		public Sequence(OpModeBase opMode)
		{
			super(opMode);
		}

		@Override
		protected void queueJobs()
		{
			Drivetrain drivetrain = opMode.getBehavior(Drivetrain.class);
			execute(drivetrain, new Drivetrain.Trace(-1f, 0.35f));
		}
	}
}
