# Platform

Official Jetson Orin Nano stack and access rules for hardware missions. Build and deploy against these versions; do not assume Gazebo or lab kits match the flight board.


## Software & Versions

The flight board is a Jetson Orin Nano running a Linux image.
- **NVIDIA L4T**: `36.4.4`
- **JetPack**: `6.2.1`

Compile and load models:
- CUDA: `12.6`
- CUDA toolkit: `12.6.11`
- CUDA runtime / compiler: `12.6.68`
- cuDNN: `9.3.0.75`

## Sensors

**Optical Flow (MTF-01p)**  
SkyTrack does not provide an SDK for permission access to optical-flow raw data. Teams must not rely on direct MTF-01p reads for custom ranging or recognition. Flow is reserved for the flight stack (Pixhawk 6C), not for team pipelines.

**Camera (IMX219-83)**  
ISP parameters are fixed on the flight image. Exposure, white balance, and other ISP controls are not adjustable through the team pipeline. Design vision under fixed ISP.

## Safety intervention

If the aircraft leaves a safe trajectory or risks collision, a safety pilot may take over with the Radiomaster RC.

RC takeover **aborts** the mission. There is no resume after re-stabilization; the flight counts as cancelled.
