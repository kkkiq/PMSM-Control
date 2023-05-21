# PMSM-Control
Trimming, Numerical Linearization and Control example project. Project made along with Control Systems laboratory class (2022.2) at Instituto Federal Fluminense (IFF).

On week 1 folder, there is an axample on how to use numerical analysis to search for a desirable operation point (also known as trimming process).

On week 2, I undergo through the numerical linearization process. Prof. Christopher Lum has an amazing source of info about this process [here](https://www.youtube.com/watch?v=1VmeijdM1qs).

On week 3, the trimming and numerical linearization process is done again, but with more complex techniques after the teacher's evaluation. the fmincon() command is used to search for the operation point, and the computation of the matrices are done automatically. At the end, the non-linear PMSM model and its linearization around the operation point are compared.

![lin_vs_nonlin](https://github.com/kkkiq/PMSM-Control/assets/85909385/0605a084-2f69-40ac-9d73-c4a59dd1786e)

On week 4, we design the controller. The architecture chosen is PI. We use the linear model as plant, and use the state augmentation method to produce a state feedback vector. For more info, check chapter 12 of Ogata's Modern Control Engineering Handbook.

![Id_step](https://github.com/kkkiq/PMSM-Control/assets/85909385/53e6b083-49b2-4678-a098-b3349a4e7007)
![Iq_step](https://github.com/kkkiq/PMSM-Control/assets/85909385/4a4110d7-e813-4864-9840-2c17a3bf2824)
![omega_step](https://github.com/kkkiq/PMSM-Control/assets/85909385/9bae3f81-0d3c-48bd-a162-89a7af411b87)


On week 5, we briefly try to introduce the concepts of monte carlo simulation, running the nonlinear controlled simulation with noises on its measurements, inputs, and parameters.

