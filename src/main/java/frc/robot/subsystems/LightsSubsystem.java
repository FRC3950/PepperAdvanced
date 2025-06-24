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

  // public void setLEDOverride(boolean override, AnimationType animation) {
  //   overrideActive = override;
  //   if (override && animation != null) {
  //     candle.setControl(whiteStrobe);
  //   }
  // }

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

  StrobeAnimation greenStrobe = createStrobeAnimation(kGreen, kSlot0StartIdx, kSlot0EndIdx, 0, 5);
  StrobeAnimation whiteStrobe = createStrobeAnimation(kWhite, kSlot0StartIdx, kSlot0EndIdx, 1, 6);
  RainbowAnimation rainbow = createRainbowAnimation(kSlot0StartIdx, kSlot0EndIdx, 2, 10.0);
  EmptyAnimation fullClear0 = new EmptyAnimation(0);
  EmptyAnimation fullClear1 = new EmptyAnimation(1);
  EmptyAnimation fullClear2 = new EmptyAnimation(2);
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
    // if (overrideActive) {
    //   return;
    // }

    boolean intakeState = mailbox.somethingInIntake();
    boolean elevatorActive = elevator.elevatorLeadMotor.getPosition().getValueAsDouble() > 0;

    // Only update when a condition changes
    if (intakeState != wasSomethingInIntake || elevatorActive != wasElevatorActive) {
      if (intakeState) {
        // If something is in intake, use strobe animation when elevator goes up.
        if (elevatorActive) {
          candle.setControl(greenStrobe);
        } else {
          candle.setControl(new EmptyAnimation(0));
          candle.setControl(solidGreen);
        }
      } else {
        candle.setControl(new EmptyAnimation(0));
        candle.setControl(solidRed);
      }
      wasSomethingInIntake = intakeState;
      wasElevatorActive = elevatorActive;
    }

    // Instead of caching and reusing the animation instance, create a new one each tim

    wasSomethingInIntake = intakeState;
    wasElevatorActive = elevatorActive;
  }
}
