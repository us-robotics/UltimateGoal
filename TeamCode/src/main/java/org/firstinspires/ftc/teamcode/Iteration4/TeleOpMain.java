package org.firstinspires.ftc.teamcode.Iteration4;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Behaviors.*;

import java.util.List;

import FTCEngine.Core.Auto.ConfigOption;
import FTCEngine.Core.Behavior;
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
		behaviorList.add(new WobbleGrabber(this));
		behaviorList.add(new InertialMeasurementUnit(this));
		behaviorList.add(new TeleOpSequences(this));
//		behaviorList.add(new ColorSensors(this));
	}

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{

	}

	@Override
	public void loop()
	{
		debug.addData("FPS", 1f / time.getDeltaTime());
		super.loop();
	}
}