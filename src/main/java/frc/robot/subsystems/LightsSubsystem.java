package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MailBox;
import frc.robot.subsystems.elevator.*;

public class LightsSubsystem extends SubsystemBase {
  public static final CANdle candle = new CANdle(10, "CANivore");

  private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
  private static final RGBWColor kGreen = RGBWColor.fromHSV(Degrees.of(125), 1, 0.40);
  private static final RGBWColor kRed = RGBWColor.fromHSV(Degrees.of(0), 1, 0.75);
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
  private static final int kSlot0EndIdx = 92;

  public enum AnimationType {
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
  private final Elevator elevator;
  private boolean wasSomethingInIntake = false;
  private boolean wasElevatorActive = false;
  private boolean overrideActive = false;
  private boolean rainbowActive = false;
  private AnimationType currentAnimationType = AnimationType.None;

  public void setLEDOverride(boolean override, AnimationType animation) {
    overrideActive = override;
    if (override && animation != null) {
      candle.setControl(whiteStrobe);
    }
  }

  // Update constructor to receive Mailbox
  public LightsSubsystem(MailBox mailbox, Elevator elevator) {
    this.mailbox = mailbox;
    this.elevator = elevator;
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.LED.StripType = StripTypeValue.GRB;
    candleConfiguration.LED.BrightnessScalar = 0.75;
    candleConfiguration.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    candle.getConfigurator().apply(candleConfiguration);

    // candle.setControl(rainbow);
    // rainbowActive = true;
  }

  StrobeAnimation greenStrobe = createStrobeAnimation(kGreen, kSlot0StartIdx, kSlot0EndIdx, 3, 5);
  StrobeAnimation whiteStrobe = createStrobeAnimation(kWhite, kSlot0StartIdx, kSlot0EndIdx, 2, 6);
  // RainbowAnimation rainbow = createRainbowAnimation(kSlot0StartIdx, kSlot0EndIdx, 1, 18.0);
  EmptyAnimation fullClear = new EmptyAnimation(0);
  SolidColor solidGreen = new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kGreen);
  SolidColor solidRed = new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kRed);

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

  @Override
  public void periodic() {
    if (overrideActive) {
      return;
    }

    boolean intakeState = mailbox.somethingInIntake();
    boolean elevatorActive = elevator.elevatorLeadMotor.getPosition().getValueAsDouble() > 0;

    AnimationType desiredType;
    if (!intakeState) {
      desiredType = AnimationType.Fire; // maps to red
    } else if (intakeState && elevatorActive) {
      desiredType = AnimationType.Strobe; // maps to green strobe
    } else {
      desiredType = AnimationType.RgbFade; // maps to solid green
    }

    // Instead of caching and reusing the animation instance, create a new one each time.
    switch (desiredType) {
      case Fire:
        candle.setControl(createSolidRedAnimation());
        break;
      case Strobe:
        candle.setControl(createGreenStrobeAnimation());
        break;
      case RgbFade:
        candle.setControl(createSolidGreenAnimation());
        break;
      default:
        candle.setControl(fullClear);
        break;
    }

    wasSomethingInIntake = intakeState;
    wasElevatorActive = elevatorActive;
  }

  // Create new animation instances each time to reset state:
  private SolidColor createSolidRedAnimation() {
    // Return a new instance of a solid red animation; adjust parameters as needed.
    return new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kRed);
  }

  private StrobeAnimation createGreenStrobeAnimation() {
    // Create a fresh green strobe animation instance with desired parameters.
    return createStrobeAnimation(kGreen, kSlot0StartIdx, kSlot0EndIdx, 3, 5);
  }

  private SolidColor createSolidGreenAnimation() {
    // Return a new instance of a solid green animation.
    return new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kGreen);
  }
}
