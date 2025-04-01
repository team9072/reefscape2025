package frc.robot.commands.autos.chooser;

import choreo.util.ChoreoAlert;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.commands.autos.AutoPathSegments.AutoPathSegment;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Predicate;
import java.util.stream.Stream;

class TrajectoryChooser<V extends AutoPathSegment> implements Sendable {
  private static final String NONE_NAME = "None";
  private static final Alert selectedNonexistentTrajectory =
      ChoreoAlert.alert("Selected a trajectory that isn't an option", AlertType.kError);

  private String selected = NONE_NAME;

  private final Map<String, V> optionValueMap;
  private String[] options = new String[] {NONE_NAME};

  public TrajectoryChooser(V[] trajectories) {
    optionValueMap = new HashMap<>();

    for (V trajectory : trajectories) {
      optionValueMap.put(trajectory.displayName(), trajectory);
    }

    filterOptions((v) -> true);
  }

  public void filterOptions(Predicate<V> filterPredicate) {
    Stream<String> keys =
        optionValueMap.entrySet().stream()
            .filter((entry) -> filterPredicate.test(entry.getValue()))
            .map(Map.Entry::getKey);
    options = keys.toArray((size) -> new String[size + 1]);

    options[options.length - 1] = NONE_NAME;
    Arrays.sort(
        options,
        (a, b) -> {
          boolean aNone = a == NONE_NAME;
          boolean bNone = b == NONE_NAME;

          if (aNone && bNone) {
            return 0;
          } else if (aNone) {
            return 1;
          } else if (bNone) {
            return -1;
          } else {
            return a.compareTo(b);
          }
        });
  }

  public void select(String selectStr) {
    if (selected.equals(NONE_NAME)
        || Arrays.asList(options).contains(selectStr) && optionValueMap.containsKey(selectStr)) {
      selectedNonexistentTrajectory.set(false);
      selected = selectStr;
    } else {
      selectedNonexistentTrajectory.set(true);
      selected = NONE_NAME;
    }
  }

  public void select(V selected) {
    select(selected != null ? selected.displayName() : NONE_NAME);
  }

  private String getSelectedName() {
    if (!optionValueMap.containsKey(selected) || !Arrays.asList(options).contains(selected)) {
      return NONE_NAME;
    } else {
      return selected;
    }
  }

  public V getSelected() {
    String filteredSelection = getSelectedName();

    if (filteredSelection == NONE_NAME) {
      return null;
    } else {
      return optionValueMap.get(selected);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("String Chooser");
    builder.publishConstBoolean(".controllable", true);
    builder.publishConstString("default", NONE_NAME);
    builder.addStringArrayProperty("options", () -> options, null);
    builder.addStringProperty("selected", null, this::select);
    builder.addStringProperty("active", this::getSelectedName, null);
  }
}
