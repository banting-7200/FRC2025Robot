// Package //
package frc.robot.Subsystems;

// Imports //
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

// Subsystem //
public class ShuffleboardSubsystem {
  // Static Data //
  private static ShuffleboardSubsystem instance;
  // Instance Data //
  private ShuffleboardLayout layout;
  private ShuffleboardTab tab;
  private List<String> entryNames = new ArrayList<String>();
  private List<GenericEntry> entries = new ArrayList<GenericEntry>();
  private SendableChooser<String> autos;
  private SimpleWidget lights;

  // Constructor //
  private ShuffleboardSubsystem() {
    instance = this;
  }

  // Static Methods //
  /**
   * Gets the singleton instance of the subsystem
   *
   * @return the singleton instance
   */
  public static synchronized ShuffleboardSubsystem getInstance() {
    return instance == null ? new ShuffleboardSubsystem() : instance;
  }

  // #region Base Methods //
  /**
   * Sets which tab to put values to
   *
   * @param tabName The tab to put values to
   */
  public void setTab(String tabName) { // sets which tab to put values to.
    tab = Shuffleboard.getTab(tabName);
  }

  /**
   * Sets which layout to put values to. Putting values of the same name in different layouts may
   * mess with code.
   *
   * @param layoutName The name of the layout
   * @param x The X-Axis of the size of the layout
   * @param y The Y-Axis fo the size of the layout
   */
  public void setLayout(String layoutName, int x, int y) {
    layout = tab.getLayout(layoutName, "List Layout").withSize(x, y);
  }

  /**
   * Sets which layout to put values to. Putting values of the same name in different layouts may
   * mess with code.
   *
   * @param layoutName The name of the layout
   */
  public void setLayout(String layoutName) {
    if (layoutName == null) layout = null;
    else layout = tab.getLayout(layoutName, "List Layout");
  }

  /**
   * Creates or sets a double on the shuffleboard
   *
   * @param name Name of the entry
   * @param value Value of the entry
   */
  public void setNumber(
      String name, double value) { // creates or sets a double on the shuffleboard.
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) { // If this entry does not exist
      if (layout != null) { // if theres no layout selected
        entry = layout.add(name, value).withWidget(BuiltInWidgets.kTextView).getEntry();
      } else {
        entry = tab.add(name, value).withWidget(BuiltInWidgets.kTextView).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setDouble(value);
    }
  }

  /**
   * Creates or sets a double on the shuffleboard
   *
   * @param name Name of the entry
   * @param value Value of the entry
   * @param x X-Position of the tab
   * @param y Y-Position of the tab
   */
  public void setNumber(String name, double value, int x, int y) {
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) {
      if (layout != null) {
        entry =
            layout
                .add(name, value)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(x, y)
                .getEntry();
      } else {
        entry =
            tab.add(name, value).withWidget(BuiltInWidgets.kTextView).withPosition(x, y).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setDouble(value);
    }
  }

  /**
   * Creates or sets a double on the shuffleboard with a specific widget
   *
   * @param name Name of the entry
   * @param value Value of the entry
   * @param type Specific Widget
   */
  public void setNumber(
      String name, double value, BuiltInWidgets type) { // creates or sets a double on the
    // shuffleboard with a specific widget.
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) {
      if (layout != null) {
        entry = layout.add(name, value).withWidget(type).getEntry();
      } else {
        entry = tab.add(name, value).withWidget(type).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setDouble(value);
    }
  }

  /**
   * Creates or sets a boolean on the shuffleboard
   *
   * @param name The name of the entry
   * @param value The value of the entry
   */
  public void setBoolean(String name, boolean value) {
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) {
      if (layout != null) {
        entry = layout.add(name, value).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
      } else {
        entry = tab.add(name, value).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setBoolean(value);
    }
  }

  /**
   * Gets a double from the shuffleboard
   *
   * @param name Name of the entry
   * @return The double from the shuffleboard
   */
  public double getNumber(String name) { // returns a double from the shuffleboard.
    int index = entryNames.indexOf(name);
    if (index == -1) {
      setNumber(name, 0);
      return 0;
    } else {
      return entries.get(index).getDouble(0);
    }
  }

  /**
   * Gets a boolean from the shuffleboard
   *
   * @param name Name of the entry
   * @return The boolean from the shuffleboard
   */
  public boolean getBoolean(String name) { // returns a boolean from the shuffleboard.
    int index = entryNames.indexOf(name);
    if (index == -1) {
      setBoolean(name, false);
      return false;
    } else {
      return entries.get(index).getBoolean(false);
    }
  }

  // #endregion
  // #region Swerve //
  /**
   * Create a named list layout of swerve motor and direction values.
   *
   * @param name Name of the widget
   * @param speed Swerve Speed
   * @param direction Swerve Direction
   */
  public void setSwerve(String name, double speed, double direction) {
    setLayout(name, 2, 2);
    setNumber(
        name + " Speed", speed, BuiltInWidgets.kDial); // todo:see if max and mins needs to be set.
    setNumber(name + " Direction", direction, BuiltInWidgets.kGyro); // same here
    setLayout(null);
  }

  // #endregion
  // #region PID //
  /**
   * Update PID in Shuffleboard
   *
   * @param name Name of the widget
   * @param p P-Value
   * @param i I-Value
   * @param d D-Value
   * @param f F-Value
   * @param iz IZ-Value
   */
  public void setPID(
      String name,
      double p,
      double i,
      double d,
      double f,
      double iz) { // create a named list layout of
    // PID values.
    setTab("PID");
    setLayout(name, 2, 4);
    setNumber(name + " P", p, 0, 0);
    setNumber(name + " I", i, 0, 1);
    setNumber(name + " D", d, 0, 2);
    setNumber(name + " F", f, 0, 3);
    setNumber(name + " IZ", iz, 0, 4);
    setLayout(null);
  }

  /**
   * Gets an array of PID values stored under the name they were originally set with.
   *
   * @param name The name of the widget
   * @return Returns the PID values: double[] {p, i, d, f, iz}
   */
  public double[] getPID(
      String name) { // return an array of PID values stored under the name they were originally set
    // with.
    double p = getNumber(name + " P");
    double i = getNumber(name + " I");
    double d = getNumber(name + " D");
    double f = getNumber(name + " F");
    double iz = getNumber(name + " IZ");
    return new double[] {p, i, d, f, iz};
  }

  // #endregion
  // #region Custom Widget
  /**
   * Creates a button which automatically runs the command it was set with.
   *
   * @param name The name of the command
   * @param command The command which will be run
   */
  public void newCommandButton(
      String name,
      Command command) { // creates button which automatically runs the command it was set with.
    tab.add(name, command);
  }

  /**
   * Creates drop down containing autos, doesn't add any functionality to the basic function, just
   * contains everything in the subsystem.
   *
   * @param inAutos
   */
  public void newAutoChooser(
      SendableChooser<String>
          inAutos) { // creates drop down containing autos, doesn't add any functionality to the
    // basic function, just contains everything in the subsystem
    setTab("Pre-Match");
    autos = inAutos;
    tab.add("Autos", autos).withSize(2, 1);
    autos.setDefaultOption("No Auto Selected", "(MIDDLE) Basic Auto");
  }

  /**
   * Gets auto from the drop down
   *
   * @return The auto
   */
  public String getAuto() { // returns auto from drop down;
    return (autos.getSelected());
  }

  /**
   * Adds camera to shufffleboard
   *
   * @param name Widget Name
   * @param camera Camera
   * @param url URL
   */
  public void addCamera(String name, String camera, String url) {
    tab.addCamera(name, camera, url).withSize(2, 2); // doesnt add any functionalaty again
  }

  /**
   * Puts text to the dashboard
   *
   * @param name Entry Name
   * @param text Text to be written
   */
  public void setText(String name, String text) { // puts text to the dashboard
    GenericEntry entry;
    int index = entryNames.indexOf(name);
    if (index == -1) {
      if (layout != null) {
        entry = layout.add(name, text).getEntry();
      } else {
        entry = tab.add(name, text).getEntry();
      }
      entryNames.add(name);
      entries.add(entry);
    } else {
      entries.get(index).setString(text);
    }
  }

  /**
   * Gets text from the shuffleboard
   *
   * @param name Name of the entry
   * @return The text
   */
  public String getText(String name) { // returns text from shuffleboard
    int index = entryNames.indexOf(name);
    if (index == -1) {
      setText(name, " ");
      return " ";
    } else {
      return entries.get(index).getString(" ");
    }
  }

  /**
   * Custom colour widget
   *
   * @param name Name of the widget
   * @param colour Colour of the widget
   */
  public void setColour(String name, Color colour) { // custom colour widget
    try {
      GenericEntry entry;
      int index = entryNames.indexOf(name);
      Map<String, Object> map = Map.of("colorWhenTrue", colour.toHexString());
      if (layout != null) {
        entry =
            layout
                .add(name, true)
                .withProperties(map)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();
      } else {
        entry =
            tab.add(name, true)
                .withProperties(map)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();
      }
      if (index == -1) {
        entryNames.add(name);
        entries.add(entry);
      }

      // lights.withProperties(Map.of("colorWhenTrue", colour.toHexString()));
    } catch (IllegalArgumentException e) {
      System.out.println("Set Color Error, EXEPTION: " + e);
    }
  }

  // #endregion
  // #region Algae Intake //
  /**
   * Updates the values of the arm on the shuffleboard
   *
   * @param name The name of the widget
   * @param armStatus The status of the arm
   */
  public void setArms(String name, boolean armStatus) {
    setTab("Arm");
    setLayout(name, 2, 4);
    setBoolean(name + "Arm Status", armStatus);
    setLayout(null);
  }

  /**
   * Gets the status of the arm on the shuffleboard, will return false if entry doesn't exist.
   *
   * @param name The name of the widget
   * @return The status of the arm
   */
  public boolean getArms(String name) {
    int index = entryNames.indexOf(name);
    if (index == -1) {
      return getBoolean(name);
    } else {
      return entries.get(index).getBoolean(false);
    }
  }

  // #endregion
  // #region Lift //
  public void setLiftLevel(String name, int level) {
    setTab("Lift");
    setLayout(name, 2, 4);
    setNumber(name + "Lift Level", level);
    setLayout(null);
  }

  public int getLiftLevel(String name) {
    int index = entryNames.indexOf(name);
    if (index == -1) {
      return (int) getNumber(name);
    } else {
      return (int) entries.get(index).getInteger(0);
    }
  }

  // #endregion //
  // #region Coral Intake //
  public void setCoralPivot(String name, double pivot) {
    setTab("Coral Intake");
    setLayout(name, 2, 4);
    setNumber(name, pivot);
    setLayout(null);
  }

  public double getCoralPivot(String name) {
    int index = entryNames.indexOf(name);
    if (index == -1) {
      return (int) getNumber(name);
    } else {
      return (int) entries.get(index).getDouble(0);
    }
  }
  // #endregion //
  // #region
}
