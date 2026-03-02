# STM32_LSM6DSM_Filtered_IMU_and_Visualizer

I demonstrated how to use an IMU sensor on an STM32 using I2C communication with the LSM6DSM sensor library. Data from the IMU sensor can be visualized using the visualizer program located in the Tools folder. To use visualizer, you must create env and install some libraries:

```bash
python -m .env env
    
pip install pyserial pygame PyOpenGL PyOpenGL_accelerate   
```

Example visualizer usage:

`python visualizer_3d.py --port COM3 --baud 115200`


If you don't have a sensor and card, you can use the simulation project. Within the simulation, different scenario conditions can be run to observe how the library and filter structure behave. To use the simulation, build.bat can be run through the Visual Studio 2022 terminal, or compilation can be performed using makefile.

Windows MSVC Build:
`.\build.bat`

Makefile build:
`make`


The sensor library I used and the necessary information (functions, filters, etc.) can be found at the link below.

[LSM6DSM_Hardware_Abstracted_Library](https://github.com/K4lender/LSM6DSM_Hardware_Abstracted_Library)