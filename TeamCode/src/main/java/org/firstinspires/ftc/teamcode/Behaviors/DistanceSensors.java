package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;

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

		front = hardwareMap.get(DistanceSensor.class, "frontDistance");
		back = hardwareMap.get(DistanceSensor.class, "backDistance");
	}

	private DistanceSensor front;
	private DistanceSensor back;

	@Override
	public void update()
	{
		super.update();

		opMode.debug.addData("Distance", getDistance());
	}

	public float getDistance()
	{
		float frontDistance = (float)front.getDistance(DistanceUnit.CM);
		float backDistance = (float)back.getDistance(DistanceUnit.CM);

		final float Threshold = 500f;

		if (frontDistance > Threshold) return backDistance;
		if (backDistance > Threshold) return frontDistance;

		return (frontDistance + backDistance) / 2f;
	}
}
