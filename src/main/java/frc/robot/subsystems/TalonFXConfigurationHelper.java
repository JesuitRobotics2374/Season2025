package frc.robot.subsystems;

import java.io.File;
import java.lang.reflect.Field;
import java.util.Scanner;
import java.util.StringTokenizer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXConfigurationHelper {

    private final File file; // The configuration file to change the motor
    private final TalonFX motorController; // The motor
    private final TalonFXConfiguration configuration; // Configuration to change the motor

    public TalonFXConfigurationHelper(TalonFXConfiguration configuration, File file, TalonFX motorController) {
        this.file = file;
        this.configuration = configuration;
        this.motorController = motorController;
    }

    // The function to read the configuration file and change the configuration of
    // the motor

    public void setConfiguration() throws Exception {
        Field[] fields = TalonFXConfiguration.class.getFields(); // Gets all variables in the TalonFXConfiguration class
        Scanner scanner = new Scanner(file);
        String fileValueString = "";

        while (scanner.hasNextLine()) {
            fileValueString += scanner.nextLine() + "\n";
        }

        scanner.close();

        StringTokenizer st = new StringTokenizer(fileValueString);

        for (Field field : fields) {
            if (!field.equals(fields[0])) {
                Object obj = field.get(configuration);
                for (int i = 0; i < obj.getClass().getFields().length; i++) {

                    // Parses the configuration file and changes each variable in the
                    // TalonFXConfiguration class

                    String preKey = st.nextToken();
                    String key = preKey.substring(0, preKey.length() - 1);
                    String value = st.nextToken();

                    Field subField = obj.getClass().getField(key); // Gets the variables in the previously gotten
                                                                   // variables
                    subField.set(obj, convertString(value, subField.getType())); // Sets the variables to the correct
                                                                                 // values
                }
            }
        }
        motorController.getConfigurator().refresh(configuration); // Changes the configuration of the motor

    }

    @SuppressWarnings("unchecked")
    private <T extends Enum<T>> Object convertString(String value, Class<?> clazz) {
        try {
            return Integer.valueOf(value);
        } catch (Exception e) {

        }
        try {
            return Double.valueOf(value);
        } catch (Exception e) {

        }
        try {
            return Enum.valueOf((Class<T>) clazz, value);
        } catch (Exception e) {
        }
        if (value.contains("true")) {
            return true;
        } else if (value.contains("false")) {
            return false;
        }
        return value;
    }

}