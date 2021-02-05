package org.firstinspires.ftc.teamcode.Behaviors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import FTCEngine.Core.Behavior;
import FTCEngine.Core.OpModeBase;
import FTCEngine.Math.Mathf;
import FTCEngine.Math.Vector2;
import FTCEngine.Math.Vector3;

public class ColorSensors extends Behavior
{
	/**
	 * NOTE: Do not configure the electronics in the constructor, do them in the awake method!
	 *
	 * @param opMode
	 */
	public ColorSensors(OpModeBase opMode)
	{
		super(opMode);
	}

	@Override
	public void awake(HardwareMap hardwareMap)
	{
		super.awake(hardwareMap);

		left = hardwareMap.colorSensor.get("leftColor");
		right = hardwareMap.colorSensor.get("rightColor");

		left.enableLed(true);
		right.enableLed(true);
	}

	private ColorSensor left;
	private ColorSensor right;

	@Override
	public void update()
	{
		super.update();

		Vector3 rightColor = getColor(right);
		Vector3 leftColor = getColor(left);

		opMode.debug.addData("Left Line", getLineUpper() + " // " + getSaturationBrightness(leftColor));
		opMode.debug.addData("Right Line", getLineLower() + " // " + getSaturationBrightness(rightColor));
	}

	public Line getLineUpper()
	{
		return getLine(left);
	}

	public Line getLineLower()
	{
		return getLine(right);
	}

	private Line getLine(ColorSensor sensor)
	{
		Vector3 color = getColor(sensor);
		Vector2 sb = getSaturationBrightness(color); //Saturation and brightness/value

		final float SaturationThreshold = 45f;
		final float BrightnessThreshold = 2e5f;

		if (sb.x > SaturationThreshold) return Line.COLOR;
		return sb.y > BrightnessThreshold ? Line.WHITE : Line.NONE;
	}

	private static Vector2 getSaturationBrightness(Vector3 color)
	{
		float cmax = Math.max(color.x, Math.max(color.y, color.z));
		float cmin = Math.min(color.x, Math.min(color.y, color.z));

		float saturation = Mathf.almostEquals(cmax, 0f) ? 0f : (cmax - cmin) / cmax;
		return new Vector2(saturation * 100f, cmax * 100f);
	}

	private static Vector3 getColor(ColorSensor sensor)
	{
		return new Vector3(sensor.red(), sensor.green(), sensor.blue());
	}

	public enum Line
	{
		NONE,
		COLOR,
		WHITE
	}
}
