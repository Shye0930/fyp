# fyp

| Proj # 	| CCDS24-0982 	|
|---	|---	|
| Acad Yr/ Sem 	| 2024/2 	|
| Proj Title 	| Smart Navigation Device for the Visually Impaired 	|
| Proj Summary 	| This project aims to build wearable devices that can guide people with visual impairment in indoor navigation such as home, retail establishments. The core technology for achieving this is visual localization and mapping. When the user enters the environment, a map will be downloaded to the device. The wearable devices will run visual localization that tracks the movement of the user over time. In addition, the wearable device will a provide audio/tactile feedback to the user to ensure safe and comfortable navigation. 	|



## Running ORB SLAM 3 via ros2
Within the FYP folder run
```sh
ros2 run orbslam3 stereo /home/shye/Desktop/projects/fyp/ORB_SLAM_3_COMMUNITY/Vocabulary/ORBvoc.txt /home/shye/Desktop/projects/fyp/config/stereo/Esp32s.yaml true

or 

ros2 run orbslam3 stereo ORB_SLAM_3_COMMUNITY/Vocabulary/ORBvoc.txt config/stereo/Esp32s.yaml true

```

## Debug info used through the repo

[INFO]: For informational messages, especially when the code is doing something expected but noteworthy (e.g., successful initialization, a specific mode being activated).

[DEBUG]: For messages related to debugging specific issues or temporary print statements you might remove later.

[WARN]: For potential issues that don't stop execution but might lead to problems (e.g., a default value being used because a parameter was missing).

[ERROR]: For critical errors that prevent the code from functioning correctly.

[TODO]: For features or improvements that need to be implemented.

[FIXME]: For known bugs that need fixing.

[HACK]: For a workaround that might not be the cleanest solution but gets the job done for now.

[NOTE]: For important design decisions or caveats.