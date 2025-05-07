## 📊 Endpoints

### `http://192.168.43.1/pid_panel`
- **GET**  
  Display the PID control panel (UI or overview).

---

### `http://192.168.43.1/pid`
- **GET**  
  Get current PID values.

---

### `http://192.168.43.1/debug`
- **GET**  
  Change or get the current debug type.

---

### `http://192.168.43.1/battery`
- **GET**  
  Get battery voltage.

---

### `http://192.168.43.1/error`
- **GET**  
  Retrieve current error code.

---

### `http://192.168.43.1/calibration_data`
- **GET**  
  Get calibration data.

---

## ✈️ Takeoff Throttle

### `http://192.168.43.1/takeoff`
- **GET**  
  Get the current `manual_takeoff_throttle` value.

### `http://192.168.43.1/takeoff?set=<value>`
- **SET**  
  Set `manual_takeoff_throttle`.

  - Range: `1400 < value < 1600`
  - Default: `0`

---

## 📈 Pressure for Height

### `http://192.168.43.1/press`
- **GET**  
  Get the pressure used to calculate height.

### `http://192.168.43.1/press?set=<value>`
- **SET**  
  Set pressure for height calculation.

  - Example: `22` → 2.2 meters, `1.5` → 15 cm
  - Default: `1.5`

---

## ⚙️ Throttle Diff

### `http://192.168.43.1/throttle_diff`
- **GET**  
  Get the current `throttle_diff` value.

### `http://192.168.43.1/throttle_diff?set=<value>`
- **SET**  
  Set `throttle_diff`.

  - Default: `1530`

---

## 🛠 Notes

- All parameters should be set using proper query format.
- Ensure you are connected to the correct Wi-Fi/network (`192.168.43.1`) to access these endpoints.

---

## 📧 Contact

For any issues or questions, feel free to open an issue or contact the maintainer.
