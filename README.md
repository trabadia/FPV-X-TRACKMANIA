**FPV Drone** is an Openplanet plugin for Trackmania 2020 that simulates acrobatic FPV drone physics using the game’s free camera.

## Requirements

* **Gamepad**: A controller is mandatory to pilot the drone.
* **Camera 7**: The physics engine activates automatically when switching to the free camera (default key `7`).

## Controls

The plugin uses the standard configuration by default:
* **Left Stick**: Throttle (Y-axis) and Yaw (X-axis).
* **Right Stick**: Pitch (Y-axis) and Roll (X-axis).

*Note: All axes can be inverted and deadzones adjusted in the plugin settings.*

## Key Features

### Advanced Physics Engine
The simulation includes adjustable parameters for motor thrust, gravity, linear/quadratic drag, angular inertia, and motor lag. It also features a gyroscopic coupling toggle for added realism.

### Practice & Recovery Tools
* **Record & Rewind**: Record your flight in real-time. Hold the Rewind key to go back in time. You can maintain stick inputs during the rewind to seamlessly resume your trajectory upon release.
* **Checkpoint System**: Save your current position with a single press and return to it instantly. A long press clears the checkpoint.
* **Respawn**: Instantly reset the drone to its starting position.

### Presets
Save and load your favorite physics settings using three dedicated slots. A pre-configured "5-inches Freestyle" preset, inspired by *Uncrashed* settings, is included.

### CSV Export for Rendering

1. Record a flight and stop the recording.
2. Click the **"→ CSV"** button in the HUD.
3. The file is saved to `OpenplanetNext/PluginStorage/FPVDrone/trajectory.csv`.
4. Use [FPV Clip](https://utils.tmtas.exchange/fpvclip.html) to convert the data for use in the Trackmania MediaTracker.

## Configuration

Access all settings via the Openplanet menu (`F3`):
* **Physics Tab**: Adjust flight behavior.
* **Binds Tab**: Assign keys and buttons for Record, Rewind, Checkpoint, and Respawn.
* **Overlay**: Toggle and customize the stick movement display.
