# 🛰️ ESP32S3 Satellite-Ground Communication System (Academic Project)

This project simulates a **satellite-to-ground station communication link** using **ESP32-S3-WROOM-1U** development boards. Developed for the 6th-semester course, *Design of Communications Systems and Digital Signal Processing*, the system integrates wireless communication, sensor telemetry, autonomous power, and digital signal processing (image compression).

---

## I. System Architecture & Setup

The core functionality relies on two ESP32-S3 modules configured in an Access Point-Client architecture.

### A. Communication Protocol and Configuration

* **Topology:** One ESP32-S3 acts as the **Ground Base (Access Point)**, and the other as the **Satellite (Client)**.
* **Protocol:** **TCP/IP** is used to manage the link, ensuring reliable and ordered data transmission.
* **Link Management:** Communication uses plaintext messages with structured prefixes (e.g., `SAT:MSG:<payload>`) and confirmation messages (`ACK`) to maintain stability and prevent message loss.
* **Development Environment:** The system code was developed and deployed primarily in the **Arduino environment** for rapid prototyping.

### B. Hardware Subsystems

The satellite module integrates various components to simulate real-world satellite functions:

* **Sensor Telemetry (I2C):** Sensors for monitoring simulated environmental and operational status include:
    * **AHT20:** Measures temperature and humidity.
    * **MPU6050:** Measures inertial data (acceleration and orientation).
    * **INA219:** Monitors electrical power parameters (current, voltage, power).
* **Actuators:** Includes a Servomotor and an RGB LED to emulate physical responses like reorientation or signaling.
* **Power Management:** Utilizes a **solar panel and LiPo battery** setup with a **boost converter** to regulate voltage from $\approx 3V$ to $5V$, simulating autonomous power operation.

---

## II. Analysis & Some Results

### A. Channel Characterization (RSSI & CSI)

Experimental analysis was conducted in a 6m x 6m room to characterize the wireless channel.

* **RSSI Analysis:** The system's effective communication range was determined to be between **100 and 120 meters**.
* **Path Loss Modeling:** Applied the Close-In Free Space Reference Distance model to the measured RSSI data, resulting in a path loss exponent of **$n=2.01$**, which closely matches the theoretical free-space value ($n=2$).
* **CSI Analysis:** Channel State Information (CSI) measurements demonstrated that the **amplitude standard deviation** is a sensitive metric for detecting human presence/obstruction, increasing significantly from **0.48-0.98** (no obstruction) to **1.27-1.95** (with obstruction).

### B. Image Compression

* **Methodology:** Implemented color quantization using the **K-means clustering algorithm** to reduce the number of unique colors in an image, minimizing data size.
* **Channel Focus:** Compression was focused on the **Y (Luminance)** channel of the YCbCr color space, as it is the most critical component for visual perception. The optimal number of clusters was determined to be **$K=64$** using the elbow method.
* **Implementation Note:** The version of the image sent is the reconstructed image obtained *after* K-means compression. The clustering process was implemented for learning purposes—it was a great way to explore data reduction! However, transmitting the true compressed form (centroids and cluster IDs) and reconstructing it on the ground station was reserved for future work. This simplified the transmission but means we sent the full-sized "reconstructed" picture instead of the super-tiny data payload.

### C. Overall Performance

* **Data Throughput:** Successfully transmitted a reconstructed image in approximately **35 seconds**, divided into **390 data chunks**.
* **Power Consumption:** The ESP32 averaged approximately **820 mW** during operation.

## III. Files

The project files are organized into dedicated directories for code, hardware design, and signal processing tools:

| Category | Directory Path | Description |
| :--- | :--- | :--- |
| **Ground Code** | `ESP32SAT/blob/main/GroundConnect/GroundConnect.ino` | Arduino sketch for the Ground Base (Access Point). |
| **Satellite Code** | `ESP32SAT/blob/main/Satellitev4/Satellitev4.ino` | Arduino sketch for the Satellite (Client). |
| **Image Processing** | `ESP32SAT/tree/main/IMG` | MATLAB scripts for the K-means clustering algorithm (IMG). |
| **User Interface** | `ESP32SAT/blob/main/GUI/InterfazGrafica.mlapp` | The main file for the MATLAB Graphical User Interface (GUI). |
| **Documentation** | `ESP32SAT/blob/main/TE2011B%20FReport.pdf` | If you like to read, and want to read more :P |
