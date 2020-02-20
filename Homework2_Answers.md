# Answers for Homework 2

## Test results

| Test | FV     | AV     | ET (deg) | EX (m)  | EY(m)  | OT(deg)   | OX(m)   | OY(m)   | FT(deg)  | FX(m)   | FY(m)  | GT(deg)   | GX(m)   | GY(m)  | DT = (OT-GT)/20 | DX = (OX-GX)/10 | DY     |
|------|--------|--------|----------|---------|--------|-----------|---------|---------|----------|---------|--------|-----------|---------|--------|-----------------|-----------------|--------|
| CCW  | 1.0000 | 2.8400 | 0.0000   | 0.0000  | 0.0000 | -44.6334  | 0.0000  | 0.0000  | -12.4905 | 0.0000  | 0.0000 | 120.0003  | 0.0000  | 0.0000 | -8.2317         | 0.0000          | 0.0000 |
| CCW  | 0.7000 | 1.9880 | 0.0000   | 0.0000  | 0.0000 | 132.2387  | 0.0000  | 0.0000  | -14.7823 | 0.0000  | 0.0000 | 139.9736  | 0.0000  | 0.0000 | -0.0068         | 0.0000          | 0.0000 |
| CW   | 1.0000 | 2.8400 | 0.0000   | 0.0000  | 0.0000 | 61.1919   | 0.0000  | 0.0000  | 12.4905  | 0.0000  | 0.0000 | -105.0002 | 0.0000  | 0.0000 | 8.3096          | 0.0000          | 0.0000 |
| CW   | 0.7000 | 1.9880 | 0.0000   | 0.0000  | 0.0000 | -131.7803 | 0.0000  | 0.0000  | 14.4958  | 0.0000  | 0.0000 | -174.9813 | 0.0000  | 0.0000 | 2.1601          | 0.0000          | 0.0000 |
| FWD  | 1.0000 | 0.2200 | 0.0000   | 2.0000  | 0.0000 | 0.0000    | 1.8500  | 0.0040  | 0.0000   | 1.9980  | 0.0000 | 0.0000    | 1.7526  | 0.0000 | 0.0000          | 0.0097          | 0.0000 |
| FWD  | 0.7000 | 0.1540 | 0.0000   | 2.0000  | 0.0000 | 0.0000    | 1.9270  | 0.0000  | 0.0000   | 1.9860  | 0.0000 | 0.0000    | 1.8542  | 0.0000 | 0.0000          | 0.0073          | 0.0000 |
| BKWD | 1.0000 | 0.2200 | 0.0000   | -2.0000 | 0.0000 | 0.0000    | -1.8540 | -0.0670 | 0.0000   | -1.9980 | 0.0000 | 0.0000    | -1.8212 | 0.0000 | 0.0000          | -0.0033         | 0.0000 |
| BKWD | 0.7000 | 0.1540 | 0.0000   | -2.0000 | 0.0000 | 0.0000    | -1.9280 | 0.0030  | 0.0000   | -1.9870 | 0.0000 | 0.0000    | -1.9050 | 0.0000 | 0.0000          | -0.0023         | 0.0000 |

## F.007

- 6. Yes. The forward model does not take the dynamcis of the robot into account. The robot is not able to achieve the target speed immediately. Using the encoders eliminated the difference between the velocity command and how much the wheel actually rotated.
- 7. Yes.

## F.009

- [Visualization in Rviz](https://github.com/ME495-Navigation/main-assignment-shangzhouye/raw/master/figures/nuturtle_rviz.mkv)
- [Running in real world](https://github.com/ME495-Navigation/main-assignment-shangzhouye/raw/master/figures/nuturtle_real_world.mov)

- FV: 0.7	
- linear velocity = 0.1540
- angular velocity = 1.9880
- total_distance =  6.828 m
- x_error = 0.49 m
- y_error = 0.02m							