# Barebone-ThirdController

Fork of [Puresoul's Barebone](https://github.com/puresoul/Barebone) OpenVR controller driver.

This was an experiment in 2023 for trying to enable legacy mixed reality with older games that expect a third controller instead of a tracker type device, while still being able to use LIV for avatar and tracker position.

# Install and Usage

- Compile the driver and VC Manager app.
- Copy the barebones folder to SteamVR's drivers folder. 
- Edit steamvr.vrsettings to have "activateMultipleDrivers" set to true.
- Run SteamVR, the status window should show an inactive tracker.
- Run VC Manager and press a device to follow (e.g. LIV's virtual tracker).
- A controller should appear in the same place as that selected driver.
- Some games with a externalcamera.cfg file will enable legacy quads for mixed reality capture.