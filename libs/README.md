This folder exists because the DistanceSensor lib [isn't published correctly](https://github.com/REVrobotics/2m-Distance-Sensor/issues/15) and requires a really weird installation method, which isn't ideal for reproducibility, version control, and CI builds

# Steps taken

- download the 2m-Distance-Sensor-SDK-2024.0.4.zip from https://github.com/REVrobotics/2m-Distance-Sensor/releases/tag/v2024.0.4
- from the zip, copy the `maven/com` folder to the new folder `libs` (I don't know where to really put this tbh)
    - NOTE: I omitted the cpp and driver stuff to save some MB
- include the jar file in the build.gradle

```gradle
dependencies {
    // ...
    implementation files('libs/com/revrobotics/frc/DistanceSensor-java/0.4.0/DistanceSensor-java-0.4.0.jar')
    // ...
}
```