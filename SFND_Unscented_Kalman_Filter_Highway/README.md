

# Unscented Kalman Filter(UKF)
![KFGIF](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/KFGIF.gif)

[Watch Video]([path/to/video.mp4](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/KalmanFilterProject.mp4))

- The answer has to be derived from why EKF is limited and what advantage we have with UKF.
- The Extended Kalman Filter (EKF) applies a first-order Taylor series linearization to non-linear process and measurement models using Jacobians.This linearization introduces approximation errors, which can degrade estimation accuracy—especially for highly non-linear systems or poor initial estimates.
- Computing and maintaining Jacobians can be mathematically complex, error-prone, and sometimes infeasible for complex or black-box models.
- The Unscented Kalman Filter (UKF) avoids explicit linearization. Instead, it uses a deterministic sampling technique called the Unscented Transform, which propagates a set of carefully chosen sigma points through the true non-linear models resulting in better accuracy and robustness than EKF for non-linear systems.

In this project, we use the **CTRV (Constant Turn Rate and Velocity)** motion model as the **process model** for object state prediction.
The state is predicted using the CTRV model, and the predicted state is then updated using measurements from multiple sensors:
- **LiDAR**, which provides **linear position measurements**:  
  \((p_x, p_y)\)
- **RADAR**, which provides **non-linear measurements**:  
  range \((\rho)\), bearing \((\phi)\), and range rate \((\dot{\rho})\)

A **UKF** framework is used, where the prediction step propagates sigma points through the CTRV process model, and the update step applies the appropriate measurement model depending on the sensor type.
- The UKF project follows the RoadMap below:
[![UKF Road Map](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/UKF%20Road%20Map)](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/UKF%20Road%20Map)

*UKF Road Map*


### 1.Generate Sigma Points:
- As per the **CTRV model**, the state vector consists of **5 state elements** \((n_x = 5)\).  
  A common rule of thumb in UKF is to generate **\(2n_x + 1\)** sigma points to represent the state distribution.
- Instead of directly propagating the mean and covariance through the non-linear process model—which can distort the Gaussian distribution—the UKF propagates a **set of deterministically chosen sigma points** through the non-linear model.
- These sigma points are selected such that they **capture the mean and covariance** of the state distribution. After passing through the non-linear process model, the transformed sigma points are used to **reconstruct the predicted mean and covariance**, providing a more accurate approximation of the true distribution.
- The following formula shows how the sigma points are generated:
  ![GSP1](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/GSP1.png)
  ![GSP2](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/GSP2.png)
- While generating sigma points, we initially missed incorporating **process noise**.  
  To account for this, the **state vector is augmented** to include the process noise terms, modeled as **stochastic parameters**.

- In the CTRV model, the process noise consists of:
  - **Longitudinal (linear) acceleration noise**
  - **Yaw (angular) acceleration noise**

- By augmenting the original state vector with these noise components, the **augmented state dimension** becomes:
  \[
  n_{aug} = 7
  \]
- Consequently, the number of sigma points generated is:
  \[
  2n_{aug} + 1 = 15
  \]
- These **augmented sigma points** are then propagated through the CTRV process model, ensuring that the effects of process noise are properly captured in the predicted mean and covariance.
![GSP3](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/ASP1.png)

##### 2. Predict Sigma Points
- Each sigma point is propagated through the **non-linear CTRV process model** to obtain the corresponding **predicted sigma points**.
- During this step, the effects of both the system dynamics and the **process noise** (from the augmented state) are incorporated.
- Special care is taken to handle edge cases, such as near-zero yaw rate, to ensure numerical stability.
![PSP](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/PSP.png)

##### 3. Predict Mean and Covariance
- Using the predicted sigma points, the **predicted state mean and covariance** are reconstructed using the associated **UKF weights**.
![PMC2](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/PMC2.png)
- This reconstructed mean and covariance represent the **prior (predicted) state estimate** at the current time step.
- From this point onward, the predicted mean and covariance are used as the **input for the measurement update step**.
![PMC1](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/PMC1.png)

#### 4. Predict Measurement
- In the measurement prediction step, the sigma points are propagated through the **measurement model** instead of the process model.
- To avoid regenerating sigma points, the **predicted sigma points from the process step** are reused directly.
![MP1](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/MP1.png)
![MP2](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/MP2.png)
- Each predicted sigma point is transformed from **state space** into **measurement space** using the appropriate sensor measurement model:
  - **LiDAR**: linear mapping to \((p_x, p_y)\)
  - **RADAR**: non-linear mapping to \((\rho, \phi, \dot{\rho})\)
- The transformed sigma points are then used to compute the **predicted measurement mean and measurement covariance**, which are required for the update step.
- Angle normalization is applied where necessary (e.g., RADAR bearing) to ensure consistency.
- There is **no need to augment the state with measurement noise** during the measurement prediction step, unlike in the process prediction.
- Measurement noise is assumed to be **additive** and **independent** of the state. Therefore, its effect is incorporated by **directly adding the measurement noise covariance matrix** \(R\) to the predicted measurement covariance.
![MP3](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/MP3.png)
- We compute Measurement Mean(Zpred) and Measurement covariance S out of the same,

#### 5. Measurement Update
- In this step, the **actual sensor measurement** \(z\) is incorporated to correct the predicted state.
- The **measurement residual** is computed as:
  \[
  z - Z_{pred}
  \]
![UU1](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/UU1.png)
- Unlike the standard Kalman Filter, the **Kalman gain** in the UKF is computed using the **cross-correlation matrix** between:
  - the predicted sigma points in **state space**, and  
  - the transformed sigma points in **measurement space**.
- Using this cross-correlation and the measurement covariance, the Kalman gain is computed as per the UKF formulation.
- The predicted state mean and covariance are then updated using the Kalman gain and the measurement residual, yielding the **posterior state estimate**.[X_ and P_ (Updated state and covaiance)]
![UU2](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/UU2.png)

#### 6. Normalized Innovation Squared (NIS)
- **Normalized Innovation Squared (NIS)** is used as a metric to evaluate the **consistency of the filter**, indicating whether the process and measurement noise have been **overestimated or underestimated**.

![NIS1](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/NIS1.png)
![NIS2](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/NIS2.png)
- NIS is computed using the measurement residual and the measurement covariance:
![NIS3](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/NIS3.png)
- The computed NIS value is compared against the **Chi-square (\(\chi^2\)) distribution** with degrees of freedom equal to the **measurement dimension**.
- ![NIS4](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/NIS4.png)
- If the NIS value lies:
  - **Above** the Chi-square threshold → noise is likely **underestimated**
  - **Below** the Chi-square threshold → noise is likely **overestimated**
- By monitoring the NIS over time, we can assess and tune the filter to ensure **statistically consistent estimation performance**.
![NIS5](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/NIS5)
![NIS6](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/NIS6.png)

Following are the results of NIS from the project:
### For std_a = 2.5 and std_yaw = 2.3
### Laser NIS: 
![LASER_NIS_2_5_2_3](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/LASER_NIS_2_5_2_3.png)

### Radar NIS
![RadarNIS](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Unscented_Kalman_Filter_Highway/Pictures/RadarNIS.png)




