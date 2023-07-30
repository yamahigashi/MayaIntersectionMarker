# Maya Intersection Marker (Pre ALPHA)

A plugin for Autodesk Maya that detects and visualizes mesh intersections in the viewport.
![Sample](https://github.com/yamahigashi/MayaIntersectionMarker/blob/doc/doc/Animation.gif)


## Description

The Maya Intersection Marker is a plugin designed to streamline the process of checking for mesh intersections or 'penetrations' in animations. By automatically detecting and visualizing these intersections within Maya's viewport, this plugin reduces the time and effort required to identify and resolve these issues, ensuring smoother animations and improved efficiency in your workflow.

## Features

* Automatic detection of mesh intersections
* Real-time visualization of intersections in the viewport
* Supports complex mesh geometries
* User-friendly interface

## Installation
wip

## Usage
wip

## Build Instructions

This section is for developers who want to contribute to the project or build it from source. Here's how you can build the Maya Intersection Marker plugin:

### Prerequisites

* **CMake:** Used for build automation. You can download and install CMake from [here](https://cmake.org/download/).
* **Embree:** A high-performance ray tracing kernel library. You can download and install Embree from [here](https://github.com/embree/embree/releases).
* **Autodesk Maya SDK:** Make sure you have the SDK installed for the version of Maya you are developing for.


### Build Steps

1. **Clone the Repository:** Clone this repository to your local machine.
2. **Navigate to Project Directory:** Use your terminal to navigate to the project directory.
3. **Edit `mayaConfigure.bat`:** Edit the `mayaConfigure.bat` script to match your specific environment setup. Make sure to adjust the settings to match the location of your Autodesk Maya SDK and other necessary configurations.
4. **Run `mayaConfigure.bat`:** Run the `mayaConfigure.bat` script to create a build directory, configure the project with CMake, and build the project. 

### Troubleshooting

If you encounter any issues while building the project, please open an issue in this repository or refer to the CMake or Embree documentation.

### Additional Notes

Make sure that your system's environment variables are set up correctly for both CMake and Embree. Proper paths should be included in your system's PATH variable.


## Compatibility
- Maya 2024 windows
- Maya 2023 windows
- Maya 2022 windows
- Maya 2020 windows

## License
MIT

## Contributing

This project is in its early stages, and we warmly welcome any feedback or contributions. Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change. Ensure the tests pass before you make a request.

## Contact

yamahigashi - [anamorphobia@𝕏Twitter](https://twitter.com/anamorphobia)

Project Link: [https://github.com/yamahigashi/MayaIntersectionMarker](https://github.com/yamahigashi/MayaIntersectionMarker)
