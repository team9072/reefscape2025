package frc.robot.subsystems.questnav;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Used to handle communications with the Quest Nav software running on a Meta Quest 3S. Some
 * considerations are made within this class to match our expectations of FRC coordinate systems.
 * For instance, the Quest Nav, because it uses Unity, follows a left handed coordinate system,
 * while in FRC, we follow a right handed coordinate system.
 *
 * <p>Currently due to limitations with the Quest Nav software, only one {@link QuestNav} instance
 * is supported. An error will be sent to the Driver Station if more then one instance is made.
 */
public class QuestNavIOReal implements QuestNavIO {
  /**
   * Flag used to ensure that only one Quest Nav object is ever created, as currently only one is
   * supported.
   */
  private static boolean m_created = false;

  /** Last message received from Quest Nav. Used to prevent console spam. */
  private long lastMiso = 0;

  private final Timer setPoseDelayTimer = new Timer();

  /** Network Tables instance. */
  private final NetworkTableInstance networkTablesInstance = NetworkTableInstance.getDefault();
  /** Quest Nav network table. Used to communicate bi-directionally with Quest Nav. */
  private final NetworkTable questnavNt4Table = networkTablesInstance.getTable("questnav");
  /** Used to receive specialized updates on the state of Quest Nav. */
  private final IntegerSubscriber misoSubscriber =
      questnavNt4Table.getIntegerTopic("miso").subscribe(0);
  /**
   * Used to command Quest Nav into specific states, for instance, to force it to recenter itself to
   * its own origin and orientation.
   */
  private final IntegerPublisher mosiPublisher = questnavNt4Table.getIntegerTopic("mosi").publish();

  /**
   * Double array used to tell the Quest Nav where it is on the field when commanding a "Pose Set"
   * command over the MOSI interface. This is a 3 element array that matches a typical right hand
   * rule pose as follows:
   *
   * <ul>
   *   <li>[0] - X Coordinate, where positive X is forward.
   *   <li>[1] - Y Coordinate, where positive Y is left.
   *   <li>[2] - Yaw Rotation, where positive is CCW from a top down perspective.
   * </ul>
   */
  private final DoubleArrayPublisher resetPosePublisher =
      questnavNt4Table.getDoubleArrayTopic("resetpose").publish();
  /** The current frame count since the Quest Nav software was started. */
  private final IntegerSubscriber questFrameCountSubscriber =
      questnavNt4Table.getIntegerTopic("frameCount").subscribe(0);
  /** The Timestamp of the Quest Nav software. */
  private final DoubleSubscriber questTimestampSubscriber =
      questnavNt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  /**
   * The current position of the Quest relative to the Quest Nav's origin. If a "Pose Set" command
   * has not been sent, the origin is wherever the Quest was last re-centered. This is a 3 element
   * array that matches a typical left hand rule position as follows:
   *
   * <ul>
   *   <li>[0] - X coordinate in meters, where positive X is right.
   *   <li>[1] - Y coordinate in meters, where positive Y is up.
   *   <li>[2] - Z coordinate in meters, where positive Z is forward.
   * </ul>
   */
  private final FloatArraySubscriber questPositionSubscriber =
      questnavNt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  /**
   * The orientation of the Quest relative to the Quest Nav's origin orientation, as quaternions. If
   * a "Pose Set" command has not been sent, the orientation origin is wherever orientation the
   * Quest was when it was last re-centered.
   */
  private final FloatArraySubscriber questQuaternionSubscriber =
      questnavNt4Table
          .getFloatArrayTopic("quaternion")
          .subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  /**
   * The orientation of the Quest relative to the Quest Nav's origin orientation, as euler angles.
   * If a "Pose Set" command has not been sent, the orientation origin is wherever orientation the
   * Quest was when it was last re-centered. This is a 3 element array that follows a left hand
   * coordinate frame as listed below:
   *
   * <ul>
   *   <li>[0] - Pitch in degrees, where positive pitch is nose down.
   *   <li>[1] - Yaw in degrees, where positive yaw is CW from a top down perspective.
   *   <li>[2] - Roll in degrees, where positive roll is CCW looking at the quest from the rear
   *       (through the lenses).
   * </ul>
   */
  private final FloatArraySubscriber questEulerAnglesSubscriber =
      questnavNt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

  /** Current percentage of the battery. */
  private final DoubleSubscriber questBatteryPercentSubscriber =
      questnavNt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  /**
   * Position index used when indexing the "position" network table entry. This enum is written to
   * "correct" the Quest Nav's coordinate system to match our FRC expected standard of coordinate
   * systems.
   */
  private enum PositionIndex {
    /**
     * Represents forward being positive X. Actually the Z axis to Quest Nav (Unity).
     *
     * <p>Unit: meters
     */
    X(2),
    /**
     * Represents left being positive Y. Actually the X axis to Quest Nav (Unity). This follows a
     * left handed coordinate system and needs to be negated in order to match our FRC expected
     * standard of coordinate systems.
     *
     * <p>Unit: meters
     */
    Y(0),
    /**
     * Represents up being positive Z. Actually the Y axis to Quest Nav (Unity).
     *
     * <p>Unit: meters
     */
    Z(1);

    /** Index value to use when indexing the "position" network table entry. */
    public final int arrayIndex;

    private PositionIndex(int arrayIndex) {
      this.arrayIndex = arrayIndex;
    }
  }

  /**
   * Euler Angle index used when indexing the "eulerAngles" network table entry. This enum is
   * written to "correct" the Quest Nav's coordinate system to match our FRC expected standard of
   * coordinate systems.
   */
  private enum EulerAngleIndex {
    /**
     * Positive yaw is CW from a top down perspective. This follows a left handed coordinate system
     * and needs to be negated in order to match our FRC expected standard of coordinate systems.
     *
     * <p>Unit: degrees
     */
    YAW(1),
    /**
     * Positive pitch is nose down for the Quest.
     *
     * <p>Unit: degrees
     */
    PITCH(0), // TODO: Confirm this is correct.
    /**
     * Positive roll is CCW looking at the quest from the rear (through the lenses). This follows a
     * left handed coordinate system and needs to be negated in order to match our FRC expected
     * standard of coordinate systems.
     *
     * <p>Unit: degrees
     */
    ROLL(2); // TODO: Confirm this is correct.

    /** Index value to use when indexing the "eulerAngles" network table entry. */
    public final int arrayIndex;

    private EulerAngleIndex(int arrayIndex) {
      this.arrayIndex = arrayIndex;
    }
  }

  /** Commands received from the Quest Nav software. */
  private enum MisoCommand {
    /**
     * The Quest Nav software is currently operating as normal with no specific routine being
     * followed. Also means an error happened, but we can't tell the difference.
     */
    NOTHING(0),
    /**
     * Response back from the Quest Nav software after a {@link MosiCommand#PING} command is sent.
     */
    PING(97),
    /**
     * Response from the Quest Nav software that the commanded {@link MosiCommand#SET_POSE} routine
     * has completed.
     */
    POSE_SET(98),
    /**
     * Response from the Quest Nav software that the commanded {@link MosiCommand#RECENTER_PLAYER}
     * routine has completed.
     */
    PLAYER_RECENTERED(99),
    /**
     * Not an actual response coded into the Quest Nav software. Reserved for an unexpected state
     * from Quest Nav.
     */
    UNRECOGNIZED(-1);

    /** Command number used when sending a value across the "miso" network table entry. */
    public final int number;

    private MisoCommand(int number) {
      this.number = number;
    }

    public static MisoCommand fromRaw(long number) {
      for (var enumValue : MisoCommand.values()) {
        if (number == enumValue.number) {
          return enumValue;
        }
      }
      return UNRECOGNIZED;
    }
  }

  /** Commands to send to the Quest Nav software. */
  private enum MosiCommand {
    /**
     * Clears the current routine being ran. It is necessary to send this after a proper response
     * over the miso channel is received to ensure that the requested routine is fully resolved.
     */
    CLEAR(0),
    /**
     * Requests that the Quest Nav software commands a re-center routine to the Quest hardware. This
     * is similar to long-pressing the quest logo on a Quest controller.
     */
    RECENTER_PLAYER(1),
    /**
     * Requests that the Quest Nav software records a software defined offset to correct its
     * calculated position for future updates. This command should happen after the "resetpose"
     * network table entry is correctly configured.
     */
    SET_POSE(2),
    /** Requests that the Quest Nav software returns a {@link MisoCommand#PING} response back. */
    PING(3);

    /**
     * Command number used when checking the received value sent across the "mosi" network table
     * entry.
     */
    private final int number;

    private MosiCommand(int number) {
      this.number = number;
    }

    public void sendCommand(IntegerPublisher mosiPublisher) {
      mosiPublisher.set(this.number);
    }
  }

  /**
   * Creates a Quest Nav class meant to interface with a Quest 3S running Quest Nav over network
   * tables. Reports an error to Driver Station if an instance of {@link QuestNav} was detected to
   * have been instantiated somewhere else.
   *
   * @param positionOnRobot the position of the Quest on the robot. The origin of the Quest is
   *     roughly where the center of the user's eyes are if wearing the headset. A yaw of 0.0
   *     represents looking forward from the user's perspective if wearing the headset.
   */
  public QuestNavIOReal() {
    if (m_created) {
      DriverStation.reportError("QuestNav already instantiated somewhere else.", true);
    }

    m_created = true;
  }

  /**
   * Gets the raw yaw angle provided by the Quest Nav software. If {@link
   * QuestNav#resetPose(Pose2dU<Length>)} was used correctly to update the Quest Nav software's
   * interpretation of where the Quest is relative to the field, this position should be the yaw
   * orientation of the Quest relative to the field.
   *
   * @return the raw yaw angle of the Quest from Quest Nav
   */
  private Rotation2d getRawYaw() {
    return Rotation2d.fromDegrees(questEulerAnglesSubscriber.get()[EulerAngleIndex.YAW.arrayIndex]);
  }

  /**
   * The corrected yaw angle that matches the robot's 0.0 position for yaw.
   *
   * @return the corrected yaw angle
   */
  public Rotation2d getYaw() {
    return getRawYaw().unaryMinus().minus(QuestNavConstants.robotToQuest.getRotation());
  }

  /**
   * The raw position from the Quest Nav software. If {@link QuestNav#resetPose(Pose2dU<Length>)}
   * was used correctly to update the Quest Nav software's interpretation of where the Quest is
   * relative to the field, this position should be the position of the Quest relative to the field.
   *
   * @return the raw position of the Quest from Quest Nav
   */
  private Translation2d getRawPosition() {
    float[] oculusArr = questPositionSubscriber.get();
    return new Translation2d(
        Meters.of(oculusArr[PositionIndex.X.arrayIndex]),
        Meters.of(oculusArr[PositionIndex.Y.arrayIndex]).unaryMinus());
  }

  private QuestNavPoseObservation getObservation() {
    Rotation2d yaw = getYaw();
    Translation2d translation =
        getRawPosition().minus(QuestNavConstants.robotToQuest.getTranslation().rotateBy(yaw));

    return new QuestNavPoseObservation(new Pose2d(translation, yaw), Timer.getFPGATimestamp());
  }

  /**
   * Update routine. Currently only checks for responses from the QuestNav software and handles
   * cleaning up the MOSI interface.
   */
  private void updateNetworkTables() {
    long misoRaw = misoSubscriber.get();
    if (misoRaw != 0) {
      if (misoRaw != lastMiso) {
        System.out.println("Received MISO message from QuestNav: " + MisoCommand.fromRaw(misoRaw));
      }
      mosiPublisher.set(0);
    }
    lastMiso = misoRaw;

    // Check for delay on set pose command
    if (setPoseDelayTimer.hasElapsed(QuestNavConstants.setPoseDelay.in(Seconds))) {
      setPoseDelayTimer.stop();
      setPoseDelayTimer.reset();
      MosiCommand.SET_POSE.sendCommand(mosiPublisher);
    }
  }

  @Override
  public void updateInputs(QuestNavInputs inputs) {
    updateNetworkTables();
    inputs.latestObservation = getObservation();
    inputs.batteryPercentage = questBatteryPercentSubscriber.get();
    inputs.latestTimestamp = Seconds.of(questTimestampSubscriber.get());
    inputs.frameCount = questFrameCountSubscriber.get();
    inputs.isBusy = misoSubscriber.get() != 0;
  }

  @Override
  public void resetPose(Pose2d robotPose) {
    Translation2d robotPosition = robotPose.getTranslation();
    Rotation2d robotRotation = robotPose.getRotation();

    Translation2d robotToQuestPosition =
        robotPosition.plus(QuestNavConstants.robotToQuest.getTranslation().rotateBy(robotRotation));
    Rotation2d angleOffset = robotRotation.plus(QuestNavConstants.robotToQuest.getRotation());

    Pose2d questPose = new Pose2d(robotToQuestPosition, angleOffset);
    // Unity is left-hand rule, but that gets corrected in the Quest Nav software when setting the
    // pose. Also takes rotation in degrees.
    double[] questPoseArray = {
      questPose.getX(), questPose.getX(), questPose.getRotation().getDegrees()
    };

    resetPosePublisher.accept(questPoseArray);
    // Start the delayed timer used to stagger the command from the pose array update
    setPoseDelayTimer.start();
  }
}
