package org.firstinspires.ftc.teamcode.Iteration4;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Behaviors.CameraVision;
import org.firstinspires.ftc.teamcode.Behaviors.ColorSensors;
import org.firstinspires.ftc.teamcode.Behaviors.DistanceSensors;
import org.firstinspires.ftc.teamcode.Behaviors.Drivetrain;
import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.TeleOpSequences;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabber;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabberI5;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Behavior;
import FTCEngine.Core.Input;
import FTCEngine.Core.OpModeBase;

@TeleOp(name = "Ultimate Goal TeleOpMain")
public class TeleOpMain extends OpModeBase
{
	@Override
	public void addBehaviors(List<Behavior> behaviorList)
	{
		behaviorList.add(new Drivetrain(this));
		behaviorList.add(new Launcher(this));
		behaviorList.add(new Intake(this));
//		behaviorList.add(new WobbleGrabber(this));
		behaviorList.add(new WobbleGrabberI5(this));
		behaviorList.add(new InertialMeasurementUnit(this));
		behaviorList.add(new TeleOpSequences(this));
		behaviorList.add(new ColorSensors(this));
		behaviorList.add(new DistanceSensors(this));
	}

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{
		WobbleGrabberI5 grabber = getBehavior(WobbleGrabberI5.class);

		options.add(new ConfigOption()
		{
			@Override
			public String getLabel()
			{
				return "P";
			}

			@Override
			public String getOption()
			{
				return String.valueOf(grabber.constantP);
			}

			@Override
			public int getButtonCount()
			{
				return 2;
			}

			@Override
			public Input.Button getButton(int index)
			{
				if (index == 0) return Input.Button.DPAD_DOWN;
				if (index == 1) return Input.Button.DPAD_UP;

				throw new IllegalArgumentException();
			}

			@Override
			public void onButtonDown(Input.Button button)
			{
				switch (button)
				{
					case DPAD_DOWN: grabber.constantP -= 0.1f; break;
					case DPAD_UP: grabber.constantP += 0.1f; break;
				}
			}
		});

		options.add(new ConfigOption()
		{
			@Override
			public String getLabel()
			{
				return "I";
			}

			@Override
			public String getOption()
			{
				return String.valueOf(grabber.constantI);
			}

			@Override
			public int getButtonCount()
			{
				return 2;
			}

			@Override
			public Input.Button getButton(int index)
			{
				if (index == 0) return Input.Button.DPAD_LEFT;
				if (index == 1) return Input.Button.DPAD_RIGHT;

				throw new IllegalArgumentException();
			}

			@Override
			public void onButtonDown(Input.Button button)
			{
				switch (button)
				{
					case DPAD_LEFT: grabber.constantI -= 0.1f; break;
					case DPAD_RIGHT: grabber.constantI += 0.1f; break;
				}
			}
		});

		options.add(new ConfigOption()
		{
			@Override
			public String getLabel()
			{
				return "D";
			}

			@Override
			public String getOption()
			{
				return String.valueOf(grabber.constantD);
			}

			@Override
			public int getButtonCount()
			{
				return 2;
			}

			@Override
			public Input.Button getButton(int index)
			{
				if (index == 0) return Input.Button.LEFT_BUMPER;
				if (index == 1) return Input.Button.RIGHT_BUMPER;

				throw new IllegalArgumentException();
			}

			@Override
			public void onButtonDown(Input.Button button)
			{
				switch (button)
				{
					case LEFT_BUMPER: grabber.constantD -= 0.1f; break;
					case RIGHT_BUMPER: grabber.constantD += 0.1f; break;
				}
			}
		});
	}

	@Override
	public void loop()
	{
		debug.addData("FPS", 1f / time.getDeltaTime());
		if (hasSequence()) debug.addData("Running Sequence");

		super.loop();
	}
}
