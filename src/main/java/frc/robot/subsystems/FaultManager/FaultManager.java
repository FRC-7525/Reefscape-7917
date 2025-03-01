package frc.robot.Subsystems.FaultManager;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.Faults;

import swervelib.encoders.SwerveAbsoluteEncoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class FaultManager {

	public static AtomicReference<FaultManager> instance = new AtomicReference<>();

	private Map<String, ArrayList<Integer>> CANDeviceOrder = new HashMap<>();

	private Map<String, CANDevice> CANDeviceMap = new HashMap<>();

	public class Device {

		public Map<String, Integer> faults = new HashMap<String, Integer>();
		public String deviceName;

		public boolean alive;

		public Device(String deviceName) {
			this.deviceName = deviceName;
			this.alive = true;
		}

		public String getDeviceName() {
			return this.deviceName;
		}

		public boolean getFault() {
			return faults.size() > 0;
		}

		private void addFault(String fault) {
			if (faults.get(fault) == null) {
				faults.put(fault, 1);
			} else faults.put(fault, faults.get(fault) + 1);
		}

		private void removeFault(String fault) {
			if (faults.get(fault) == null) {
				return;
			}

			faults.put(fault, faults.get(fault) - 1);

			if (faults.get(fault) <= 0) {
				faults.remove(fault);
			}
		}

		public void updateFault(String fault, boolean value) {
			if (value) {
				this.addFault(fault);
			} else {
				this.removeFault(fault);
			}
		}
	}

	public class CANDevice extends Device {

		private SparkMax sparkMax;
		private CANcoder canCoder;

		
		public CANDeviceTypes deviceType;
		public String busName;

		public CANDevice(SparkMax sparkMax, String name) {
			super(name);
			this.deviceType = CANDeviceTypes.SPARK;
			this.sparkMax = sparkMax;
		}

		public CANDevice(CANcoder canCoder, String name) {
			super(name);
			this.deviceType = CANDeviceTypes.CANCODER;
			this.canCoder = canCoder;
		}

		public SparkMax getSparkMax() {
			if (deviceType != CANDeviceTypes.SPARK) {
				throw new Error("This device is not a SparkMax");
			}

			return this.sparkMax;
		}

		public CANcoder getCANcoder() {
			if (deviceType != CANDeviceTypes.CANCODER) {
				throw new Error("This device is not a CANcoder");
			}

			return this.canCoder;
		}
	}

	private FaultManager() {}

	public static FaultManager getInstance() {
		if (instance.get() == null) {
			instance.set(new FaultManager());
		}
		return instance.get();
	}

	public void periodic() {
		checkDevices();
		logDevices();
	}

	public void checkDevices() {
		for (String busName : CANDeviceOrder.keySet()) {
			for (int id : CANDeviceOrder.get(busName)) {
				CANDevice device = CANDeviceMap.get(busName + " " + id);

				switch (device.deviceType) {
					case SPARK:
						SparkMax spark = device.getSparkMax();
						Faults faults = spark.getFaults();

						device.updateFault("CAN Fault", faults.can);
						device.updateFault("Temperature Fault", faults.temperature);
						device.updateFault("ESC EEPROM Fault", faults.escEeprom);
						device.updateFault("Firmware Fault", faults.firmware);
						device.updateFault("Gate Driver Fault", faults.gateDriver);
						device.updateFault("Motor Type Fault", faults.motorType);
						device.updateFault("Fault Detected", faults.other);
						device.updateFault("Sensor Fault", faults.sensor);

						device.alive = faults.temperature;

						break;
					case CANCODER:
						CANcoder canCoder = device.getCANcoder();

						device.updateFault("Magnet Fault", canCoder.getFault_BadMagnet().getValue());
						device.updateFault("Booting Fault", canCoder.getFault_BootDuringEnable().getValue());
						device.updateFault("Hardware Fault", canCoder.getFault_Hardware().getValue());
						device.updateFault("Undervoltage Fault", canCoder.getFault_Undervoltage().getValue());

						device.alive = canCoder.isConnected();

						break;
					default:
						break;
				}
			}
		}
	}

	public void logDevices() {
		for (String busName : CANDeviceOrder.keySet()) {
			for (int id : CANDeviceOrder.get(busName)) {
				CANDevice device = CANDeviceMap.get(busName + " " + id);

				if (device.getFault()) {
					for (String fault : device.faults.keySet()) {
						Logger.recordOutput("FaultManager/CAN Device Faults/" + busName + "/" + device.deviceName + "(" + id + ")", fault);
					}
				}

				// System.out.println(device.alive);
				Logger.recordOutput("FaultManager/Alive CAN Devices/" + busName + "/" + device.deviceName + "(" + id + ")", device.alive);
			}
		}
	}

	/**
	 * Input an array of integers that represent the CAN IDs of the devices
	 * organized into the order that they are in the CAN chain
	 */
	public void calibrateDeviceOrder(ArrayList<Integer> deviceOrder, String busName) {
		CANDeviceOrder.put(busName, deviceOrder);
	}

	// Adding CAN Devices
	public void addDevice(SparkMax sparkMax, String name, String busName) {
		CANDeviceMap.put(busName + " " + sparkMax.getDeviceId(), new CANDevice(sparkMax, name));
	}

	public void addDevice(CANcoder encoder, String name, String busName) {
		CANDeviceMap.put(busName + " " + encoder.getDeviceID(), new CANDevice(encoder, name));
	}
}
