## SFND RADAR Target Generation And Detection

The following picture represents the complete project layout.

<img src="https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Radar_Target_Generation_And_Detection/Images/ProjectLayout.png" width="700" height="400" />

Here are the each rubrics in alignment with the same.

**1 . Using the given system requirements, design a FMCW waveform. Find its Bandwidth (B), chirp time (Tchirp) and slope of the chirp.
The defined system requirements of radar below:**

<img src="https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Radar_Target_Generation_And_Detection/Images/RadarSpecifications.png" width="700" height="200" />

The calculated Bandwidth(B), Chirp Duration(Tchirp) and slope (S)

<img src="https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Radar_Target_Generation_And_Detection/Images/BTS.png" width="700" height="200" />

FMCW ( Frequency Modulated Continuous Wave) for both transmit and recieved signal can be be defined using the wave equations below. 

<img src="https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Radar_Target_Generation_And_Detection/Images/WaveEquations.png" width="700" height="400" />

**2. Simulate Target movement and calculate the beat or mixed signal for every timestamp.**

The initial target position and constant velocity is set as below.

<img src="https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Radar_Target_Generation_And_Detection/Images/initialVelocityAndRange.png" width="700" height="200" />

Next, the range and delay_time is simulated using motion equations as below.
<img src="https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Radar_Target_Generation_And_Detection/Images/MotionEqn.png" width="700" height="200" />

Using the time stamp and simulated delay time, the Tx and Rx are generated and mixing them both using Tx*.Rx gives us the beat signal , which will be utilised by RangeFFT or 1D-FFT for range measurements

** 3. Implement the Range FFT on the Beat or Mixed Signal and plot the result.
The recieved beat signal is further processed by 1D - FFT as below, matlab has an inbuilt function which does all the maths.
The output is shown, where the peak is clear w.r.t to the range we have initially set and the value is also in the range of +/- 10m

## Image of 1D FFT.

The 2D-FFT of Range Doppler Map(RDM) is already implemented , and the result is gotten as below.
We can nicely gauge we are very close to the output, but one more step on refining this is to be done which is 2d-CFAR

## Image of 2d-FFT

Implement the 2D CFAR process on the output of 2D FFT operation, i.e the Range Doppler Map.

The 2D -CFAR - Constant False Alarm Rate is the technique which helps us in finding Dynamic threshold, that segregates noise from target peaks, considering the homogenous nature of noise around the peaks.

The process involves, 3 category of cells as picture below, 
## picture of training,guard and cut
The training cells are the choosen cells around Cell Under Test(CUT), from which the values of training cells are summed and averaged, which is
further added with an offset value to fianlize the actual threshold value. Note the gaurd cells are excluded from the calculation

The gaurd cells helps avoiding leaking of the target signal to training cells, which can impact noise calculation.

This Training_Cell and Gaurd cell window is slided across the entire RDM to dynamically calculate the threshold.

The following code snippet is CA(CellAveraging)-2DCFAR:
where, 
- The Tr - Training Cell numbers along Range, Td- Training Cell numbers along Doppler
      Gr - Gaurd Cell numbers along Range, Gd- Gaurd Cell numbers along Doppler are defined
- Offset value is set
- Corresponding window size using Training, Gaurd cells and CUT are formed by 
  Grid Size = (2Tr+2Gr+1)(2Td+2Gd+1).
- Gaurd Cell region + CUT can be defined by (2Gr+1)(2Gd+1).
- The Training cells are finally  (2Tr+2Gr+1)(2Td+2Gd+1) - (2Gr+1)(2Gd+1)
- Average of Training cells are computed and summed with the offset
- Each RDM cell signal level is checked against the threshold, if its greater than threshold output is assigned 1 else 0.
- if we continue for all the possible cells, we will get the final map as below, where the peak is clearly segregated from noise.

## Image of CFAR

Follwoing is the Contour graph in 2D for easier visualization.








