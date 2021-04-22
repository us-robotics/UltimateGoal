package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Behaviors.DrivetrainI5;
import org.firstinspires.ftc.teamcode.Behaviors.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.Behaviors.Intake;
import org.firstinspires.ftc.teamcode.Behaviors.Launcher;
import org.firstinspires.ftc.teamcode.Behaviors.TeleOpSequencesI5;
import org.firstinspires.ftc.teamcode.Behaviors.WobbleGrabberI5;

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
		behaviorList.add(new DrivetrainI5(this));
		behaviorList.add(new Launcher(this));
		behaviorList.add(new Intake(this));
		behaviorList.add(new WobbleGrabberI5(this));
		behaviorList.add(new TeleOpSequencesI5(this));
		behaviorList.add(new InertialMeasurementUnit(this));
	}

	@Override
	protected void appendConfigOptions(List<ConfigOption> options)
	{

	}

	@Override
	public void start()
	{
		super.start();
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
	}

	@Override
	public void loop()
	{
		debug.addData("FPS", 1f / time.getDeltaTime());
		if (hasSequence()) debug.addData("Running Sequence");

		super.loop();
	}
}
