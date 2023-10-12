## Solar panel azimuth and elevate angle orientation and calibration

## of azimuth to the geographical ‘N’ using MPU9250 sensor

## (Gyroscope, Accelerometer & Magneto meter)

Introduction:

The orientation of the solar panel in a particular direction towards the sun is more efficient but the condition
with ideal Solar panel is that they are stationary and because of that only for a particular time of the day are
they efficient. In order to solve this issue automatic solar tracker implemented in Solar panel is much more
beneficial and will be very efficient in gathering more Solar power and generating much electricity than its
previous ideal condition.

Methodology:

The position of the Sun in respect to a particular location is not always the same and to determine the exact
location of the sun during the day Solar Positioning Algorithm^1 (SPA) is being used.

The solar positioning algorithm calculates the position of the sun in the sky at a given time and location. The
azimuth angle is the angle between the sun and the observer, measured clockwise from the north from a
particular location. It is used to determine the direction of the sun relative to the observer’s location.

The azimuth angle is like a compass direction with North=0° and South=180°.

```
𝐴𝑧𝑖𝑚𝑢𝑡ℎ=cos−^1 {
𝑠𝑖𝑛𝛿 𝑐𝑜𝑠𝜑 − 𝑐𝑜𝑠𝛿 𝑠𝑖𝑛𝜑 cos(𝐻𝑅𝐴)
𝑐𝑜𝑠𝛼
```
### }

The azimuth is calculated from the above parameters:

𝛼 =elevation

𝜑 = latitude

𝛿 = declination

Observation:


In order to get the azimuth angle, the calculation part is difficult and slight deflection in angle of the x,y &
z axis will lead to major change in the estimation of the location of the Sun. So, to make the orientation of
the Solar plane, the x axis should be aligned directly to the magnetic “N using an actual compass.

Now, to estimate the location of the Sun the geographical ‘N’ is necessary. To align the x axis along the
geographical ‘N' will use a magnetic declination function w.r.t to the location of the observer. This
declination can be either +ve or –ve depending on the location.

```
Jorhat
Latitude: 26° 45' 0" N
Longitude: 94° 13' 0" E
JORHAT
Magnetic Declination: -0° 46'
Declination is NEGATIVE (WEST)
Inclination: 42° 23'
Magnetic field strength: 49044.6 nT
```
Here in our location (Jorhat), the magnetic declination is -0°46’

Working:

Every 3D object has a major rotation around its 3 axes namely “roll” for rotation around x axis, “pitch” for
rotation around y axis and “yaw” for rotation around z axis. For the requirement of this project only yaw and
pitch are necessary.

Yaw (rotation around z axis) will determine the azimuth angle.

Pitch (rotation around y axis) will determine the elevated angle.

_fig. Solar panel plane axis simulation in processing w.r.t to the MPU9250 sensor_

Now in order to calibrate the Azimuth angle w.r.t the Geographical North will have to change the

calculated Azimuth Angle.

```
geographicalNorth = azimuthAngle + magneticDeclination;
// Ensure the geographical north angle is within the range [0, 360) degrees
if (geographicalNorth < 0 ) {
geographicalNorth += 360 ;
}
```
# z

# y

# x


References:

1. astronomy - Consistency with calculating the Solar Azimuth Angle - Physics Stack Exchange
2. The Horizontal Coordinate System (timeanddate.com)


