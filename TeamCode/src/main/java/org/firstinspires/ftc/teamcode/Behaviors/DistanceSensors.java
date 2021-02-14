package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Vector2;

public class DistanceSensors extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public DistanceSensors(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		right = hardwareMap.get(DistanceSensor.class, "rightDistance");
		front = hardwareMap.get(DistanceSensor.class, "frontDistance");
	}

	private DistanceSensor right;
	private DistanceSensor front;

	@Override
	public void update()
	{
		super.update();

		opMode.debug.addData("Distance", new Vector2(getDistance(Side.RIGHT), getDistance(Side.FRONT)));
	}

	public float getDistance(Side side)
	{
		DistanceSensor sensor = null;

		switch (side)
		{
			case RIGHT: sensor = right; break;
			case FRONT: sensor = front; break;
		}

		return (float)sensor.getDistance(DistanceUnit.CM);
	}

	public enum Side
	{
		RIGHT,
		FRONT
	}
}
