# apriltag_pose_estimation

`apriltag_pose_estimation` is a Python library for pose estimation using AprilTags. It abstracts all the OpenCV and
AprilTag library code with a Pythonic interface designed to make tuning parameters easy. It hides much of the matrices
and other technical aspects to make things as easy as possible for beginners.

## Installation

This library currently isn't available on PyPI. However, you can perform a local installation by running:

```shell
pip install ./apriltag_pose_estimation
```

Add `-e` if you plan to modify the library.

## Basic usage

Specify an AprilTag field layout in JSON:

```json
{
  "fiducials": [
    {
      "id": 0,
      "rotation_vector": [
        -1.2091995761561456,
        1.2091995761561452,
        -1.2091995761561458
      ],
      "translation_vector": [
        0,
        0.105,
        0.56
      ]
    }
  ],
  "tag_size": 0.080,
  "tag_family": "tagStandard41h12"
}
```

Load the field:

```python
from apriltag_pose_estimation.core import load_field

with open('field.json', mode='r') as f:
    field = load_field(f)
```

Specify the parameters of the camera being used:

```python
from apriltag_pose_estimation.core import CameraParameters

camera_params = CameraParameters(fx=1329.143348,
                                 fy=1326.537785,
                                 cx=945.392392,
                                 cy=521.144703,
                                 k1=-0.348650,
                                 k2=0.098710,
                                 p1=-0.000157,
                                 p2=-0.001851,
                                 k3=0.000000)
```

Create a pose estimator:

```python
from apriltag_pose_estimation.localization import PoseEstimator
from apriltag_pose_estimation.localization.strategies import (MultiTagPnPEstimationStrategy, 
                                                              LowestAmbiguityEstimationStrategy)
estimator = PoseEstimator(
    strategy=MultiTagPnPEstimationStrategy(fallback_strategy=LowestAmbiguityEstimationStrategy()),
    field=field,
    camera_params=camera_params,
    nthreads=2,
    quad_sigma=0,
    refine_edges=1,
    decode_sharpening=0.25
)
```

Estimate a pose from a grayscale image:

```python
image = # ...image...
result = estimator.estimate_pose(image)
print(result.estimated_pose)
```

## Rendering camera position

It can be useful for troubleshooting to see where the camera is in space. For this reason, we provide a `render`
subpackage which renders the current camera position in a Qt window.

Create a `CameraPoseDisplay` and initialize with an `AprilTagField`:

```python
from apriltag_pose_estimation.localization.render import CameraPoseDisplay

display = CameraPoseDisplay(field)
```

When a new camera pose is found, update the display:

```python
display.update(estimated_pose)
```

You may need to execute the application, which can be done with `display.exec_application()`. In this case, you'll need
to update the display asynchronously, such as with a `QTimer`.
