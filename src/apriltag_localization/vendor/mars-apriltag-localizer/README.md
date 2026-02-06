# mars-apriltag-localizer

## Installation

### General

The library needs to be built from source. You'll first need to install all the
following dependencies first:

* OpenCV 4.x.x (tested on 4.12.0)
* Eigen 3.x.x (tested on 3.4.1)
* AprilTag 3.x.x (tested on 3.4.5). The provided `install_apriltag.py` script
  will automatically perform the required setup. It installs the library to
  the project directory instead of one of the standard locations to avoid
  polluting your `/usr` or `/usr/local`.

While the above is definitely the most automatic, any method will work as long
as you set your prefix path appropriately!
