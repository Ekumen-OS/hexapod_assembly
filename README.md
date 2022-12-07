# Hexapod control with ESP32

This is a program that setups 3 PCA9685 to control the 18 servos of an hexapod using the [espressif](https://github.com/espressif/esp-idf) library for ESP32.

Last version of espressif library tested with this code is Commit `dcaa753f37bd90c2ec40d24940bc1c132d9532d9`.

**Note**: If you are an ekumember, [here](https://docs.google.com/document/d/1Yp-LBHIF-5DJNLzYgTkxjoMUgCG76MQ22RJjeF30HaM/edit?usp=sharing) is a simple doc with some help.

## Prerequisites

Before building you need to setup the espressif enviroment. Follow [Standard Toolchain Setup](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html).

## Build

1. Go to the esp folder (You should have it created once you completed the prerequisites).
```sh
cd ~/esp
```

2. Clone this repo
```sh
git clone https://gitlab.com/ekumenlabs/ekumembers/ekuthon2022/hexapod_assembly
```

3. `cd hexapod_assembly`

4. Set up the environment variables

```sh
. $HOME/esp/esp-idf/export.sh
```

5. Build project

```sh
idf.py build
```

5. Flash ESP32: Follow [espressif guide](. $HOME/esp/esp-idf/export.sh).

## Important

You will see some code related to bluetooth. That is a work in progress and it is not integrated. That is why it is commented in the CMakeLists file.


