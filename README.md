# CIS441-OpenAPS

## OpenAPS
To compile and run, use the PlatformIO extension in VSCode to build and upload `main.cpp` to the Arduino controller. Outputs will appear on the serial monitor. Install libraries as necessary.

## Virtual Component
To compile, use `make`. To run, use `./main`. Install libraries as necessary

## Virtual Patient
No modifications were made to the virtual patient code besides creating the `.env` file. To see the OpenAPS project work on different patient profiles, log into Thingsboard and access the OpenAPS dashboard, which contains the patient profile in an editable widget.

## Demo
A recording of the OpenAPS project in action is in `Thingsboard Demo.mov`. The blue line shows the blood glucose level of the virtual patient, and the green line shows the insulin level.

We found that when the patient has bolus insulin intake, it tends to cancel out the fluctuations in glucose levels from meals, which results in mostly insignificant basal insulin levels very close to 0.0. Therefore, to fully display the capability of the OpenAPS project in this recording, we removed all bolus insulin intake from the virtual patient profile - so the only thing in this recording that is managing the glucose level is the basal insulin from OpenAPS.