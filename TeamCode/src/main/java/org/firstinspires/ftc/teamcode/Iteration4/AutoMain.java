package org.firstinspires.ftc.teamcode.Iteration4;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Behaviors.*;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Behavior;
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
//		behaviorList.add(new ColorSensors(this));
	}

	CameraVision.Position targetZone;

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{

	}

	@Override
	public void loop()
	{
		super.loop();

		CameraVision vision = getBehavior(CameraVision.class);

		if (vision.available())
		{
			targetZone = vision.getPosition();
			vision.closeCamera();

			switch (targetZone)
			{
				case A:
				{
					assignSequence(new SequenceA(this));
					break;
				}
				case B:
				{
					assignSequence(new SequenceB(this));
					break;
				}
				case C:
				{
					assignSequence(new SequenceC(this));
					break;
				}
			}
		}

		debug.addData("Target Zone", targetZone);
	}
}
