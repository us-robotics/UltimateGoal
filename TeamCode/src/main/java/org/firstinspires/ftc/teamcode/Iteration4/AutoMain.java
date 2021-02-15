package org.firstinspires.ftc.teamcode.Iteration4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.CameraVision;
import org.firstinspires.ftc.teamcode.Behaviors.ColorSensors;
import org.firstinspires.ftc.teamcode.Behaviors.DistanceSensors;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

@Autonomous(name = "Ultimate Goal AutoMain")
public class AutoMain extends OpModeBase
{
	@Override
	protected void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new Drivetrain(this));
		behaviorList.add(new WobbleGrabber(this));
		behaviorList.add(new Launcher(this));
		behaviorList.add(new Intake(this));
		behaviorList.add(new InertialMeasurementUnit(this));
		behaviorList.add(new CameraVision(this));
		behaviorList.add(new ColorSensors(this));
		behaviorList.add(new DistanceSensors(this));
	}

	CameraVision.Position targetZone;

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{
		options.add(new ConfigOption()
		{
			@Override
			public String getLabel()
			{
				return "Target Zone";
			}

			@Override
			public String getOption()
			{
				if (targetZone == null) return "Vision";
				return targetZone.name();
			}

			@Override
			public int getButtonCount()
			{
				return 1;
			}

			@Override
			public Input.Button getButton(int index)
			{
				if (index == 0) return Input.Button.X;
				throw new IllegalArgumentException();
			}

			@Override
			public void onButtonDown(Input.Button button)
			{
				if (targetZone == null)
				{
					targetZone = CameraVision.Position.A;
					return;
				}

				switch (targetZone)
				{
					case A: targetZone = CameraVision.Position.B; break;
					case B: targetZone = CameraVision.Position.C; break;
					case C: targetZone = null; break;
				}
			}
		});
	}

	@Override
	public void start()
	{
		super.start();

		if (targetZone == null) return;

		getBehavior(CameraVision.class).closeCamera();
		assignZone();
	}

	@Override
	public void loop()
	{
		CameraVision vision = getBehavior(CameraVision.class);

		if (vision.available())
		{
			targetZone = vision.getPosition();

			vision.closeCamera();
			assignZone();
		}

		if (targetZone != null) debug.addData("Target Zone", targetZone);

		super.loop();
	}

	private void assignZone()
	{
		if (hasSequence()) return;

		switch (targetZone)
		{
			case A: assignSequence(new SequenceA(this)); break;
			case B: assignSequence(new SequenceB(this)); break;
			case C: assignSequence(new SequenceC(this)); break;
		}
	}
}
