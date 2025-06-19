package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MailBox;

public class LightsSubsystem extends SubsystemBase {
  public static final CANdle candle = new CANdle(10, "CANivore");

  private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
  private static final RGBWColor kGreen = RGBWColor.fromHSV(Degrees.of(120), 0.9, 0.8);
  private static final RGBWColor kRed = RGBWColor.fromHSV(Degrees.of(0), 0.9, 0.9);
  private static final RGBWColor kBlue = RGBWColor.fromHSV(Degrees.of(240), 0.9, 0.8);
  private static final RGBWColor kWhite = RGBWColor.fromHSV(Degrees.of(0), 0, 1);
  private static final RGBWColor kBlack = RGBWColor.fromHSV(Degrees.of(0), 0, 0);
  private static final RGBWColor kYellow = RGBWColor.fromHSV(Degrees.of(60), 0.9, 0.8);
  private static final RGBWColor kPink = RGBWColor.fromHSV(Degrees.of(300), 0.9, 0.8);
  private static final RGBWColor kOrange = RGBWColor.fromHSV(Degrees.of(30), 0.9, 0.8);
  private static final RGBWColor kCyan = RGBWColor.fromHSV(Degrees.of(180), 0.9, 0.8);
  private static final RGBWColor kGold = RGBWColor.fromHSV(Degrees.of(50), 0.9, 0.8);
  private static final RGBWColor kBrown = RGBWColor.fromHSV(Degrees.of(30), 0.5, 0.4);

  private static final int kSlot0StartIdx = 8;
  private static final int kSlot0EndIdx = 37;

  private enum AnimationType {
    None,
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
  }

  private final MailBox mailbox;

  // Update constructor to receive Mailbox
  public LightsSubsystem(MailBox mailbox) {
    this.mailbox = mailbox;
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.LED.StripType = StripTypeValue.GRB;
    candleConfiguration.LED.BrightnessScalar = 0.75;
    candleConfiguration.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    candle.getConfigurator().apply(candleConfiguration);
  }

  StrobeAnimation greenStrobe =
      createStrobeAnimation(kGreen, kSlot0StartIdx, kSlot0EndIdx, 0, 10.0);
  EmptyAnimation fullClear = new EmptyAnimation(0);

  public Command defaultCommand() {
    return run(
        () -> {
          candle.setControl(fullClear);
          if (mailbox.somethingInIntake()) {
            candle.setControl(greenStrobe);
          } else {

          }
        });
  }

  public StrobeAnimation createStrobeAnimation(
      RGBWColor color, int startIdx, int endIdx, int slot, double frameRate) {
    return new StrobeAnimation(slot, slot)
        .withColor(color)
        .withLEDStartIndex(startIdx)
        .withLEDEndIndex(endIdx)
        .withSlot(slot)
        .withFrameRate(frameRate);
  }

  public ColorFlowAnimation createColorFlowAnimation(
      RGBWColor color, int startIdx, int endIdx, int slot, double frameRate) {
    return new ColorFlowAnimation(slot, slot)
        .withColor(color)
        .withLEDStartIndex(startIdx)
        .withLEDEndIndex(endIdx)
        .withSlot(slot)
        .withFrameRate(frameRate);
  }

  public LarsonAnimation createLarsonAnimation(
      RGBWColor color, int startIdx, int endIdx, int slot, double frameRate) {
    return new LarsonAnimation(slot, slot)
        .withColor(color)
        .withLEDStartIndex(startIdx)
        .withLEDEndIndex(endIdx)
        .withSlot(slot)
        .withFrameRate(frameRate);
  }

  public RainbowAnimation createRainbowAnimation(
      int startIdx, int endIdx, int slot, double frameRate) {
    return new RainbowAnimation(slot, slot)
        .withLEDStartIndex(startIdx)
        .withLEDEndIndex(endIdx)
        .withSlot(slot)
        .withFrameRate(frameRate);
  }

  public SingleFadeAnimation createSingleFadeAnimation(
      RGBWColor color, int startIdx, int endIdx, int slot, double frameRate) {
    return new SingleFadeAnimation(slot, slot)
        .withColor(color)
        .withLEDStartIndex(startIdx)
        .withLEDEndIndex(endIdx)
        .withSlot(slot)
        .withFrameRate(frameRate);
  }
}
