# Bayes filter
<img src="https://latex.codecogs.com/svg.image?Bel(x_t)=\eta&space;P(z_t|x_t)\int&space;P(x_t|U_{t-1},x_{t-1})&space;Bel(x_{t-1})dx_{t-1}" title="https://latex.codecogs.com/svg.image?Bel(x_t)=\eta P(z_t|x_t)\int P(x_t|U_{t-1},x_{t-1}) Bel(x_{t-1})dx_{t-1}" />


# Property of  Covariance

<img src="https://latex.codecogs.com/svg.image?Cov(x)=\Sigma&space;" title="https://latex.codecogs.com/svg.image?Cov(x)=\Sigma " />
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?Cov(Ax)=A&space;\Sigma&space;A^T&space;" title="https://latex.codecogs.com/svg.image?Cov(Ax)=A \Sigma A^T " />

<br/>
<br/>

# Product of Gaussian

<img src="https://latex.codecogs.com/svg.image?\mathcal{N}(\mu_0,\,\sigma_0^{2})&space;\mathcal{N}(\mu_1,\,\sigma_1^{2})=\mathcal{N}(\mu^\prime,\,\sigma^{\prime{2}})&space;" title="https://latex.codecogs.com/svg.image?\mathcal{N}(\mu_0,\,\sigma_0^{2}) \mathcal{N}(\mu_1,\,\sigma_1^{2})=\mathcal{N}(\mu^\prime,\,\sigma^{\prime{2}}) " />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\sigma^{\prime{2}}=\sigma_0^{2}-&space;\frac{\sigma_0^{4}}{\sigma_0^{2}&space;&plus;&space;\sigma_1^{2}}&space;" title="https://latex.codecogs.com/svg.image?\sigma^{\prime{2}}=\sigma_0^{2}- \frac{\sigma_0^{4}}{\sigma_0^{2} + \sigma_1^{2}} " />

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\mu^\prime=\mu_0&plus;&space;\frac{\sigma_0^2(\mu_1-\mu_0)}{\sigma_0^2&plus;\sigma_1^2}" title="https://latex.codecogs.com/svg.image?\mu^\prime=\mu_0+ \frac{\sigma_0^2(\mu_1-\mu_0)}{\sigma_0^2+\sigma_1^2}" />

<br/>
<br/>


We can reformulate it as:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?K=\frac{\sigma_0^2}{\sigma_0^2&plus;\sigma_1^2}" title="https://latex.codecogs.com/svg.image?K=\frac{\sigma_0^2}{\sigma_0^2+\sigma_1^2}" />


<img src="https://latex.codecogs.com/svg.image?\mu^\prime=\mu_0&plus;K(\mu_1-\mu_0)" title="https://latex.codecogs.com/svg.image?\mu^\prime=\mu_0+K(\mu_1-\mu_0)" />
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\sigma^{\prime&space;2}=\sigma_0&space;^{2}-K\sigma_0^2" title="https://latex.codecogs.com/svg.image?\sigma^{\prime 2}=\sigma_0 ^{2}-K\sigma_0^2" />



<br/>
<br/>

For multidimensional data 



<img src="https://latex.codecogs.com/svg.image?K=\Sigma_0(\Sigma_0&space;&plus;\Sigma_1)^{-1}" title="https://latex.codecogs.com/svg.image?K=\Sigma_0(\Sigma_0 +\Sigma_1)^{-1}" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\vec{\mu^\prime}=\vec{\mu_0}&plus;K(\vec{\mu_1}-\vec{\mu_0})" title="https://latex.codecogs.com/svg.image?\vec{\mu^\prime}=\vec{\mu_0}+K(\vec{\mu_1}-\vec{\mu_0})" />

<br/>
<br/>







<img src="https://latex.codecogs.com/svg.image?\Sigma^{\prime&space;}=\Sigma_0&space;^{2}-K\Sigma_0^2" title="https://latex.codecogs.com/svg.image?\Sigma^{\prime }=\Sigma_0 ^{2}-K\Sigma_0^2" />

<br/>
<br/>


# Newton Law of Motion


<img src="https://latex.codecogs.com/svg.image?x_k=x_{k-1}&plus;v_{k-1}\Delta&space;t&plus;\frac{1}{2}a\Delta&space;t^2" title="https://latex.codecogs.com/svg.image?x_k=x_{k-1}+v_{k-1}\Delta t+\frac{1}{2}a\Delta t^2" />
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?v_k=v_{k-1}&plus;a\Delta&space;t" title="https://latex.codecogs.com/svg.image?v_k=v_{k-1}+a\Delta t" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=\begin{bmatrix}x_k&space;\\v_k\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\hat{X}_k=\begin{bmatrix}x_k \\v_k\end{bmatrix}" />

<br/>
<br/>



<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=\begin{bmatrix}1&space;&&space;\Delta&space;T&space;\\0&space;&&space;&space;1\\\end{bmatrix}\hat{X}_{k-1}&plus;\begin{bmatrix}\frac{\Delta&space;t^2}{2}&space;\\\Delta&space;t\end{bmatrix}a" title="https://latex.codecogs.com/svg.image?\hat{X}_k=\begin{bmatrix}1 & \Delta T \\0 & 1\\\end{bmatrix}\hat{X}_{k-1}+\begin{bmatrix}\frac{\Delta t^2}{2} \\\Delta t\end{bmatrix}a" />
<br/>
<br/>

## Prediction (Estimation)

New mean: 
<br/>
<br/>




<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=F\hat{X}_{k-1}&plus;B\vec{U}" title="https://latex.codecogs.com/svg.image?\hat{X}_k=F\hat{X}_{k-1}+B\vec{U}" />
<br/>
<br/>

New Covariance:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?P_k=F_kP_{k-1}F_k^T&plus;Q_k" title="https://latex.codecogs.com/svg.image?P_k=F_kP_{k-1}F_k^T+Q_k" />


## Correction (Update)

<img src="https://latex.codecogs.com/svg.image?\mu_k=\underbrace{H_k}_{\text{Measurement&space;Matrix}}&space;\hat{X}_k" title="https://latex.codecogs.com/svg.image?\mu_k=\underbrace{H_k}_{\text{Measurement Matrix}} \hat{X}_k" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\Sigma_k=H_k&space;P_k&space;H_k^T&plus;&space;R_k" title="https://latex.codecogs.com/svg.image?\Sigma_k=H_k P_k H_k^T+ R_k" />

<br/>
<br/>

Fusing and knocking out <img src="https://latex.codecogs.com/svg.image?H_k" title="https://latex.codecogs.com/svg.image?H_k" /> will give us new gain:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?K^\prime=P_K&space;H_K^T(H_k&space;P_k&space;H_k^T&plus;R_k)^{-1}" title="https://latex.codecogs.com/svg.image?K^\prime=P_K H_K^T(H_k P_k H_k^T+R_k)^{-1}" />
<br/>
<br/>
which give us corrected mean:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\hat{X}_k^\prime=\hat{X}_k&space;&plus;&space;K^\prime(Z_k-H_k\hat{X}_k)" title="https://latex.codecogs.com/svg.image?\hat{X}_k^\prime=\hat{X}_k + K^\prime(Z_k-H_k\hat{X}_k)" />


and new covariance:
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?P^\prime&space;_K=P_K-K^\prime&space;H_K&space;P_k" title="https://latex.codecogs.com/svg.image?P^\prime _K=P_K-K^\prime H_K P_k" />

# Extended Kalman Filter


<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=g(X_{k-1},U_{k-1})" title="https://latex.codecogs.com/svg.image?\hat{X}_k=g(X_{k-1},U_{k-1})" />

<br/>
<br/>


For instance for a 2D robot with state <img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x&space;\\y\\\phi&space;\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x \\y\\\phi \end{bmatrix}" /> and twist command <img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;\omega&space;\\V&space;\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} \omega \\V \end{bmatrix}" /> :


<br/>
<br/>



<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\underbrace{\begin{bmatrix}\frac{V}{\omega}\sin(\phi&plus;\omega&space;\delta&space;t)&space;&space;&space;-\frac{V}{\omega}&space;\sin(\phi)&space;&space;\\-\frac{V}{\omega}&space;\cos(\phi&space;&plus;\omega&space;\delta&space;t)&space;&space;&plus;\frac{V}{\omega}&space;\cos(\phi)\\\phi\end{bmatrix}&plus;\begin{bmatrix}x&space;&space;\\y&space;\\\omega&space;\delta&space;t\end{bmatrix}}_{g(u_t,\mu_{t-1})}&space;" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\underbrace{\begin{bmatrix}\frac{V}{\omega}\sin(\phi+\omega \delta t) -\frac{V}{\omega} \sin(\phi) \\-\frac{V}{\omega} \cos(\phi +\omega \delta t) +\frac{V}{\omega} \cos(\phi)\\\phi\end{bmatrix}+\begin{bmatrix}x \\y \\\omega \delta t\end{bmatrix}}_{g(u_t,\mu_{t-1})} " />

<br/>
<br/>

First order Linearization:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?f(x)\approx&space;f(a)&plus;&space;\frac{f(a)^\prime}{1!}(x-a)^1" title="https://latex.codecogs.com/svg.image?f(x)\approx f(a)+ \frac{f(a)^\prime}{1!}(x-a)^1" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?g(X_{K-1},U_{K})\approx&space;&space;\frac{\delta&space;g(\mu_K,U_k)}{\delta&space;X_{K-1}}(X_{K-1}-\mu_{K-1})" title="https://latex.codecogs.com/svg.image?g(X_{K-1},U_{K})\approx \frac{\delta g(\mu_K,U_k)}{\delta X_{K-1}}(X_{K-1}-\mu_{K-1})" />


<br/>
<br/>

Linearization of prediction function:
<br/>
<br/>
 
<img src="https://latex.codecogs.com/svg.image?G_K=\frac{\delta&space;g(\mu_K,U_k)}{\delta&space;X_{K-1}}=J=\begin{bmatrix}\frac{\delta&space;g_1}{\delta&space;x_1}&space;&space;&...&space;&space;&\frac{\delta&space;g_1}{\delta&space;x_n}\\\vdots&space;&space;&&space;&\vdots&space;\\&space;&space;\frac{\delta&space;g_m}{\delta&space;x_1}&&space;...&space;&\frac{\delta&space;g_m}{\delta&space;x_n}&space;&space;\end{bmatrix}" title="https://latex.codecogs.com/svg.image?G_K=\frac{\delta g(\mu_K,U_k)}{\delta X_{K-1}}=J=\begin{bmatrix}\frac{\delta g_1}{\delta x_1} &... &\frac{\delta g_1}{\delta x_n}\\\vdots & &\vdots \\ \frac{\delta g_m}{\delta x_1}& ... &\frac{\delta g_m}{\delta x_n} \end{bmatrix}" />

For example for a 2D mobile robot with laser scanner:

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?Z_k=h(X_k)&plus;\delta&space;k" title="https://latex.codecogs.com/svg.image?Z_k=h(X_k)+\delta k" />



<br/>
<br/>


Linearization of update function:
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?h(X_K)\approx&space;\underbrace{&space;\frac{\partial&space;h(\mu_K)}{\partial&space;X_K}&space;}_H_K&space;(X_K-\mu_K)&space;&plus;h(\mu_k)&space;" title="https://latex.codecogs.com/svg.image?h(X_K)\approx \underbrace{ \frac{\partial h(\mu_K)}{\partial X_K} }_H_K (X_K-\mu_K) +h(\mu_k) " />



## Prediction Step
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\hat{X}_k=g(X_{k-1},U_{k-1})" title="https://latex.codecogs.com/svg.image?\hat{X}_k=g(X_{k-1},U_{k-1})" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\hat{P_K}=G_K&space;P_{K-1}G_K^T&plus;Q_K" title="https://latex.codecogs.com/svg.image?\hat{P_K}=G_K P_{K-1}G_K^T+Q_K" />
<br/>
<br/>

## Update Step
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?K=\hat{P_K}&space;H_K^T(H_K&space;\hat{P_K}&space;H_K^T&space;&plus;R_K)^{-1}" title="https://latex.codecogs.com/svg.image?K=\hat{P_K} H_K^T(H_K \hat{P_K} H_K^T +R_K)^{-1}" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\hat{X^\prime&space;_K&space;}=\hat{X&space;_K&space;}&plus;K(Z_K&space;-h(\hat{X_K}))" title="https://latex.codecogs.com/svg.image?\hat{X^\prime _K }=\hat{X _K }+K(Z_K -h(\hat{X_K}))" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?P^\prime&space;_K=P_K-K&space;H_K&space;P_K" title="https://latex.codecogs.com/svg.image?P^\prime _K=P_K-K H_K P_K" />


# Tracking Multiple Objects

Refs: [1](https://www.youtube.com/watch?v=IIt1LHIHYc4)


# Track-Level Fusion

Refs: [1](https://www.youtube.com/watch?v=r0THmp0WxJI)
 
