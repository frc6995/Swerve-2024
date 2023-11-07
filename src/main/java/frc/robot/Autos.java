package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.POIManager.POIS;
import frc.robot.driver.CommandOperatorKeypad;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.LightStripS;
import frc.robot.subsystems.ArmS.ArmPosition;
import frc.robot.subsystems.LightStripS.States;
import frc.robot.util.AllianceWrapper;
import frc.robot.util.NomadMathUtil;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Autos {
    private DrivebaseS m_drivebaseS;
    private ArmS m_armS;
    private IntakeS m_intakeS;
    private CommandOperatorKeypad m_keypad;

    /**
     * Trigger that determines whether the drivebase is close enough to its target pose to score a cube.
     */
    public final Trigger m_alignSafeToPlace;
    public final Trigger m_alignSafeToPremove;


    public Autos(DrivebaseS drivebaseS, ArmS armS, IntakeS intakeS, CommandOperatorKeypad keypad) {
        m_drivebaseS = drivebaseS;
        m_armS = armS;
        m_intakeS = intakeS;
        m_keypad = keypad;
        m_alignSafeToPlace = new Trigger(()->{
            Transform2d error = new Transform2d(
                getTargetAlignmentPose(), m_drivebaseS.getPose());
            if (isCubeSelected()) {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(3) &&
                Math.abs(error.getX()) < 0.1 &&
                Math.abs(error.getY()) < 0.1;
            } else {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(2) &&
                Math.abs(error.getX()) < 0.02 &&
                Math.abs(error.getY()) < Units.inchesToMeters(0.5);
            }
        });
        m_alignSafeToPremove = new Trigger(()->{
            Transform2d error = new Transform2d(
                getTargetAlignmentPose(), m_drivebaseS.getPose());
            if (isCubeSelected()) {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(10) &&
                Math.abs(error.getX()) < 0.5 &&
                Math.abs(error.getY()) < 0.5;
            } else {
                return
                Math.abs(error.getRotation().getRadians()) < Units.degreesToRadians(10) &&
                Math.abs(error.getX()) < 0.5 &&
                Math.abs(error.getY()) < 0.5;
            }
        });
    }

    public Pose2d getTargetAlignmentPose() {
        return POIManager.ownCommunity().get(
            (int) m_keypad.get() % 9
        ).transformBy(isCubeSelected() ? new Transform2d() : m_intakeS.getConeCenterOffset());
    }
    public ArmPosition getTargetArmPosition() {
        double node = m_keypad.get();
        if (node <= 8) {
            return ArmConstants.SCORE_HYBRID_POSITION;
        } else if (node <= 17) { // mid
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_MID_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_MID_CONE_POSITION;
            }
        } else { // high
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_HIGH_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_HIGH_CONE_POSITION;
            }
        }
    }
    public ArmPosition getPrescoreArmPosition() {
        double node = m_keypad.get();
        if (node <= 8) {
            return ArmConstants.SCORE_HYBRID_POSITION;
        } else if (node <= 17) { // mid
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_MID_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_MID_CONE_POSITION;
            }
        } else { // high
            if (node % 3 == 1) {//cube
                return ArmConstants.SCORE_MID_CUBE_POSITION;
            } else {
                return ArmConstants.SCORE_HIGH_CONE_POSITION;
            }
        }
    }
    public boolean isCubeSelected() {
        double node = m_keypad.get();
        return (node <= 8) || (node > 8 && node % 3 == 1);
    }
    public boolean isHybridSelected() {
        double node = m_keypad.get();
        return (node <= 8);
    }
    /**
     * Command factory for 
     */
    private Command alignToSelectedScoring() {
        return  m_drivebaseS.chasePoseC(this::getTargetAlignmentPose);
    }

        /**
     * Command factory for intaking a game piece.
     * @param position The arm position
     * @param isCube Whether the intake should be in cube mode or cone mode.
     * @return
     */
    public Command armIntakeCG(ArmPosition position, ArmPosition prestow, boolean isCube) {
        return 
        sequence(
            // Start intaking, and stop when a piece is detected.\
            deadline(
                waitSeconds(0.3).andThen(m_intakeS.intakeUntilBeamBreakC(isCube)).andThen(m_intakeS.intakeC(()->isCube).withTimeout(0.2)).asProxy(),
                // move to arm position while intaking.
                m_armS.goToPositionIndefiniteC(position)

            ),
            parallel(
                // Wait a bit, then pulse the intake to ensure piece collection.
                waitSeconds(isCube ? 0: 0.75).andThen(m_intakeS.intakeC(()->isCube).withTimeout(isCube ? 1.5 : 0.75)).asProxy(),
                // stow the arm
                m_armS.goToPositionC(()->prestow).andThen( m_armS.goToPositionC(()->isCube? ArmPositions.CUBE_STOW : ArmPositions.STOW)),
                run(()->LightStripS.getInstance().requestState(isCube ? States.IntakedCube : States.IntakedCone)).asProxy().withTimeout(0.75)
            )
        );
    }

    public Command armIntakeCG(ArmPosition position, boolean isCube) {
        return armIntakeCG(position, isCube? ArmPositions.CUBE_STOW : ArmPositions.STOW, isCube);
    }

    public Command armIntakeSelectedCG(ArmPosition cubePosition, ArmPosition conePosition, BooleanSupplier isCube) {
        return either(
            armIntakeCG(cubePosition, true), armIntakeCG(conePosition, false), isCube);
    }
    public Command autoScoreSequenceCG() {
        return sequence(
                m_armS.goToPositionC(this::getTargetArmPosition).until(m_keypad.leftGrid().and(m_keypad.centerGrid()).and(m_keypad.rightGrid())),
                        m_intakeS.outtakeC(this::isCubeSelected, this::isHybridSelected).withTimeout(0.4),
                        m_armS.stowC()
            )
            .deadlineWith(run(()->LightStripS.getInstance().requestState(States.Scoring)));

    }

    public Command alignScore(){
        return sequence(
                deadline(
                    sequence(
                        sequence(
                             waitUntil(m_alignSafeToPremove),
                            m_armS.goToPositionC(this::getTargetArmPosition),
                            waitUntil(m_alignSafeToPlace)
                        )//.until(m_alignSafeToPlace)//,
                        //m_armS.goToPositionC(this::getTargetArmPosition)
                    ),
                    alignToSelectedScoring().asProxy(),
                    m_intakeS.run(()->{
                        if (this.isCubeSelected() && !this.isHybridSelected()) {
                        m_intakeS.intakeCube(0.5);
                        }
                    })
                ).until(m_keypad.leftGrid().and(m_keypad.centerGrid()).and(m_keypad.rightGrid())),
                m_intakeS.outtakeC(this::isCubeSelected).withTimeout(0.4),
                m_armS.stowC() 
            );
    }

    
}
