// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.ArmConstants.*;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.ArmS.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToPositionC extends Command {
  private ArmS m_armS;
  private ArmPosition m_startPosition;
  private ArmPosition m_targetPosition;
  private List<Pair<ArmPosition, Boolean>> m_waypoints;
  private Supplier<ArmPosition> m_positionSupplier;
  private final double maxRotateLength = 0.64;
  private int currentTarget = 0;
  private boolean m_shouldFinish = true;

  /** Creates a new GoToPositionC. */
  public GoToPositionC(ArmS armS, Supplier<ArmPosition> positionSupplier) {
    m_armS = armS;
    m_positionSupplier = positionSupplier;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(m_armS);
  }

  public GoToPositionC(ArmS armS, Supplier<ArmPosition> positionSupplier, boolean shouldEnd) {
    this(armS, positionSupplier);
    m_shouldFinish = shouldEnd;
  }

  private double limitLength(double length) {
    return MathUtil.clamp(length, length, length);
  }

  private double constrainWrist(double pivotAngle, double wristRadians) {
    if(pivotAngle > Math.PI) {
      return Math.min(-0.05, wristRadians);
    }
    return wristRadians;
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("armMoving", true);
    // TODO Auto-generated method stub
    super.initialize();
    currentTarget = 0;
    m_startPosition = m_armS.getArmPosition();
    m_targetPosition = m_positionSupplier.get();

    boolean needToRetract = false;
    boolean needToStraighten = false;
    /**
     * 
     */
    double rotateLength = MathUtil.clamp(Math.min(m_startPosition.armLength, m_targetPosition.armLength),
        MIN_ARM_LENGTH, maxRotateLength);

    if (Math.abs(m_startPosition.armLength - m_targetPosition.armLength) > Units.inchesToMeters(1)) {
      needToStraighten = true;
    }

    m_armS.resetExtender();
    m_armS.resetPivot();
    m_armS.resetWrist();

    m_waypoints = new LinkedList<Pair<ArmPosition, Boolean>>();
    m_waypoints.add(Pair.of(m_startPosition, true));
    /*
     * if only wrist needed:
     * go directly to target
     * if pivot needed:
     * if
     */

    double safeWrist = Units.degreesToRadians(90);
    boolean pivotNeeded = (Math.abs(m_startPosition.pivotRadians - m_targetPosition.pivotRadians) > 0.15);
    boolean retractBeforePivot = pivotNeeded && m_startPosition.armLength > maxRotateLength;
    boolean extendAfterPivot = pivotNeeded && m_targetPosition.armLength > maxRotateLength;
    // m_waypoints.add(new ArmPosition(m_startPosition.pivotRadians, m_startPosition.armLength,
    //     MathUtil.clamp(m_startPosition.wristRadians, -safeWrist, safeWrist)));
    if (retractBeforePivot) {
      // straighten wrist

      m_waypoints.add(
        Pair.of(new ArmPosition(m_startPosition.pivotRadians, Math.min(rotateLength, m_targetPosition.armLength),
          constrainWrist(m_startPosition.pivotRadians, m_targetPosition.wristRadians)), true));
    }
    if (extendAfterPivot) {
      m_waypoints.add(
        Pair.of(new ArmPosition(
          m_targetPosition.pivotRadians, rotateLength,
          constrainWrist(m_targetPosition.pivotRadians, m_targetPosition.wristRadians)), false));
    }
    m_waypoints.add(
      Pair.of(new ArmPosition(
        m_targetPosition.pivotRadians, m_targetPosition.armLength,
        constrainWrist(m_targetPosition.pivotRadians, m_targetPosition.wristRadians)),false));
    m_waypoints.add(Pair.of(m_targetPosition,true));
    // {
    // double targetLength = m_targetPosition.armLength;
    // if (Math.abs(m_startPosition.pivotRadians - m_targetPosition.pivotRadians) >
    // 0.15) {
    // needToRetract = true;

    // // Step 1, get within the safe length interval for rotating. If we can get to
    // the target length, do it.

    // double minStartLength = m_armS.getMinLength(m_startPosition.pivotRadians);
    // double firstRetractLength = m_startPosition.armLength;
    // // if (firstRetractLength > maxRotateLength) {
    // // firstRetractLength = maxRotateLength;
    // // }
    // // if (targetLength < minStartLength) {
    // // firstRetractLength = minStartLength;
    // // }
    // firstRetractLength = MathUtil.clamp(firstRetractLength, minStartLength,
    // maxRotateLength);
    // targetLength = MathUtil.clamp(targetLength, minStartLength, maxRotateLength);
    // boolean elevatorNeeded = Math.abs(m_startPosition.armLength -
    // m_targetPosition.armLength) > 0.1;
    // double wristWhileTelescoping= MathUtil.clamp(m_targetPosition.wristRadians,
    // Units.degreesToRadians(-5), Units.degreesToRadians(5));

    // if (elevatorNeeded) {
    // m_waypoints.add(new ArmPosition(
    // m_startPosition.pivotRadians,
    // m_startPosition.armLength,
    // wristWhileTelescoping
    // ));
    // }
    // if (elevatorNeeded || ) {

    // }
    // m_waypoints.add(new ArmPosition(
    // m_startPosition.pivotRadians,
    // firstRetractLength,
    // wristWhileTelescoping
    // ));
    // m_waypoints.add(new ArmPosition(
    // m_targetPosition.pivotRadians,
    // targetLength,
    // wristWhileTelescoping));
    // }

    // //step 3, extend/retract to target length
    // m_waypoints.add(m_targetPosition);
    // }
    // }

  }

  public void execute() {

    if (isAtSetpoint(m_targetPosition)) {
      currentTarget = m_waypoints.size() - 1;
    } else if (isAtSetpoint(m_waypoints.get(currentTarget).getFirst(),
    m_waypoints.get(currentTarget).getSecond()
    )) {
      if (currentTarget < m_waypoints.size() - 1) {
        currentTarget++;
      }
    }

    var currentTargetPosition = m_waypoints.get(currentTarget).getFirst();
    m_armS.setExtendLength(currentTargetPosition.armLength);
    m_armS.setAngle(currentTargetPosition.pivotRadians);
    m_armS.setWristAngle(constrainWrist( m_armS.getArmPosition().pivotRadians, currentTargetPosition.wristRadians));
  }

  public boolean isFinished() {

    double extensionError = Units.inchesToMeters(0.5);
    double pivotError = Units.degreesToRadians(3);
    double wristError = Units.degreesToRadians(5);
    var actualPosition = m_armS.getArmPosition();
    var atSetpoint = (Math.abs(m_targetPosition.armLength - actualPosition.armLength) < extensionError
        && Math.abs(m_targetPosition.pivotRadians - actualPosition.pivotRadians) < pivotError
        && Math.abs(
          constrainWrist(actualPosition.pivotRadians, m_targetPosition.wristRadians)
         - actualPosition.wristRadians) < wristError);
    return atSetpoint;
  }

  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("armMoving", false);
  }

  private boolean isAtSetpoint(ArmPosition setpoint, Boolean wristRequired) {

    var actualPosition = m_armS.getArmPosition();
    var atSetpoint = (Math.abs(setpoint.armLength - actualPosition.armLength) < Units
        .inchesToMeters(2)
        && Math.abs(setpoint.pivotRadians - actualPosition.pivotRadians) < 0.2
        && (!wristRequired || Math.abs(
          constrainWrist(actualPosition.pivotRadians, setpoint.wristRadians)
         - actualPosition.wristRadians) < Units.degreesToRadians(5)));
    return atSetpoint;
  }

  private boolean isAtSetpoint(ArmPosition setpoint) {
    return isAtSetpoint(setpoint, true);
  }
}