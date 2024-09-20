# HandiClick

## Build
To build this repository, you need ESP-IDF v5.1.4.

### Enable `arduino-esp32` component
The `components` directory is execluded from tracking, so you have to add `arduino-esp32` component.
```
$ mkdir components
$ cd components
$ git clone https://github.com/espressif/arduino-esp32.git
$ cd arduino-esp32
$ git submodule update --init --recursive 
```
And also, in esp-idf directory, you have to run the following command:
```
$ git submodule update --init --recursive
```