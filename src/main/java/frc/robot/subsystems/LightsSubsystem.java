package frc.robot.subsystems;


import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.*;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mailbox;

public class LightsSubsystem extends SubsystemBase {
  public static final CANdle candle = new CANdle(10, "CANivore");

  public static final Color orange = new Color(255, 25, 0);
  public static final Color black = new Color(0, 0, 0);
  public static final Color yellow = new Color(242, 60, 0);
  public static final Color purple = new Color(184, 0, 185);
  public static final Color white = new Color(255, 230, 220);
  public static final Color green = new Color(56, 209, 0);
  public static final Color blue = new Color(8, 32, 255);
  public static final Color red = new Color(255, 0, 0);
  public static final Color maroon = new Color(128, 0, 0);

  private final Mailbox mailbox;

  // Update constructor to receive Mailbox
  public LightsSubsystem(Mailbox mailbox) {
    this.mailbox = mailbox;
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    LightsSubsystem.candle.configLEDType(CANdle.LEDStripType.GRB);
    LightsSubsystem.candle.configV5Enabled(true);
    LightsSubsystem.LEDSegment.MainStrip.startIndex = 0;
    LightsSubsystem.LEDSegment.MainStrip.segmentSize = 50;
    LightsSubsystem.LEDSegment.MainStrip.animationSlot = 2;
  }

  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  public Command defaultCommand() {
    return run(
        () -> {
          LEDSegment.MainStrip.fullClear();
          if (Mailbox.getSomethingInIntake()) {
            LEDSegment.MainStrip.setColor(green);
          } else {
            LEDSegment.MainStrip.setColor(maroon);
          }
        });
  }

  public Command clearSegmentCommand(LEDSegment segment) {
    return runOnce(
        () -> {
          segment.clearAnimation();
          segment.disableLEDs();
        });
  }

  public static enum LEDSegment {
    MainStrip(0, 50, 2);

    public int startIndex = 0;
    public int segmentSize = 50;
    public int animationSlot = 2;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setColor(Color color) {
      clearAnimation();
      candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
    }

    private void setAnimation(Animation animation) {
      candle.animate(animation, animationSlot);
    }

    public void fullClear() {
      clearAnimation();
      disableLEDs();
    }

    public void clearAnimation() {
      candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(black);
    }

    public void setFlowAnimation(Color color, double speed) {
      setAnimation(
          new ColorFlowAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              Direction.Forward,
              startIndex));
    }

    public void setFadeAnimation(Color color, double speed) {
      setAnimation(
          new SingleFadeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setBandAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              BounceMode.Front,
              3,
              startIndex));
    }

    public void setStrobeAnimation(Color color, double speed) {
      setAnimation(
          new StrobeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setRainbowAnimation(double speed) {
      setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
    }
  }

  public static class Color {
    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }
}
