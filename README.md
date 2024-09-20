# HandiClick

## Build

### Install ESP-IDF
To build this repository, you need ESP-IDF v5.1.4.
```
$ sudo apt-get install git wget flex bison gperf python3 python3-pip python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
$ cd
$ mkdir esp
$ cd esp
$ git clone --recursive https://github.com/espressif/esp-idf.git
$ git checkout v5.1.4
$ git submodule update --init --recursive
```

### Enable `arduino-esp32` component
The `components` directory is execluded from tracking, so you have to add `arduino-esp32` component.
```
$ mkdir components
$ cd components
$ git clone https://github.com/espressif/arduino-esp32.git
$ cd arduino-esp32 
```

### 