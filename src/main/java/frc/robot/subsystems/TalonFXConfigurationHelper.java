package frc.robot.subsystems;

import java.io.File;
import java.lang.reflect.Field;
import java.util.Scanner;
import java.util.StringTokenizer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXConfigurationHelper {

    private final File file = new File("/home/lvuser/deploy/talonfx-configs.txt");
    private final TalonFXConfiguration configuration;

    public TalonFXConfigurationHelper(TalonFXConfiguration configuration) {
        this.configuration = configuration;
    }

    public void checkConfiguration() throws Exception {
        Field[] fields = TalonFXConfiguration.class.getFields();
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
                    String preKey = st.nextToken();
                    String key = preKey.substring(0, preKey.length() - 1);
                    String value = st.nextToken();

                    Field subField = obj.getClass().getField(key);
                    subField.set(obj, convertString(value, subField.getType()));
                }
            }
        }
        // System.out.println(config);

    }

    private Object convertString(String value, Class clazz) {
        try {
            return Integer.valueOf(value);
        } catch (Exception e) {

        }
        try {
            return Double.valueOf(value);
        } catch (Exception e) {

        }
        try {
            return Enum.valueOf(clazz, value);
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