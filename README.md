# Autopilot_Drone_in_Construction

This is a mobile application for autopilot UAVs on construction sites. <br />
This application is designed to handle the material/instrument supply mission using a low-cost micro UAV.


This application is built based on [DJI Mobile SDK for Android](https://github.com/dji-sdk/Mobile-SDK-Android). <br />


## Getting Started

### Installation

We integrate the [Monodepth2](https://openaccess.thecvf.com/content_ICCV_2019/html/Godard_Digging_Into_Self-Supervised_Monocular_Depth_Estimation_ICCV_2019_paper.html) algorithm into the system in order to avoid unpredicted collisions during flight. <br />
* Download the Monodepth2 from [here](https://github.com/nianticlabs/monodepth2).
* Extract the `monodepth2-master` into `app/src/main/java/pythonPackage/monodepth2` directory.
* Download the `mono_640x192` precomputed disparity predictions from the given link in their repository.
* Extract the file into the `monodepth2/models`
