# Resumen Dual IMU Ranging

## Datasets
Para este estudio se utilizaron los siguientes datasets disponibles públicamente:
* Dataset PyShoe: https://github.com/utiasSTARS/pyshoe
* Dataset HelmetPoser: https://lqiutong.github.io/HelmetPoser.github.io/

## Algoritmos de Orientación
Para este estudio se consideraron los siguientes algoritmos:

* Filtro de Kalman Extendido (EKF)
* Filtro de Mahony
* Filtro de Madgwick
* Filtro Complementario

Se utilizaron las implementaciones de la librería AHRS (Attitude Heading Reference System) en Python. Descripciones detalladas de los algoritmos pueden encontrarse en la documentación correspondiente: https://ahrs.readthedocs.io/en/latest/index.html. En todos los diagramas y resultados presentados en este resumen se utiliza el EKF como algoritmo de estimación de orientación.

## Simulación de unidad de medida inercial

En este estudio se trabajó con dos simuladores de IMU:

1. La librería AHRS presenta una implementación propia de un simulador de IMU cuya descripción se encuentra detallada en la documentación correspondiente (https://ahrs.readthedocs.io/en/latest/sensors.html).
2. Se utilizó el simulador de IMU proveniente del siguiente repositorio https://github.com/xioTechnologies/IMU-Simulator.

Para poder validar la fiabilidad de las simulaciones de IMU provenientes de la librería AHRS se propuso el sistema presentado en la siguiente figura.

<p align="center">
  <img src="DiagramaIMU-SimuladorAHRS-Comparador.jpg" alt="Figura 1" style="max-width: 100%;"><br>
  <strong>Figura 1:</strong> Esquema de validación del simulador de IMU con la librería AHRS
</p>

Para poder validar la fiabilidad de las simulaciones de IMU provenientes del repositorio de código antes mencionado (punto 2), se propuso el sistema presentado en la Figura 1. Se usó el simulador de IMU de AHRS para generar mediciones artificiales del magnetómetro consistentes con los datos restantes.

<p align="center">
  <img src="DiagramaIMU-Simuladores-Comparadores.jpg" alt="Figura 2" style="max-width: 100%;"><br>
  <strong>Figura 2:</strong> Esquema de validación del simulador de IMU del repositorio de GitHub
</p>

Suponiendo que se tienen $q_{\text{real}}$ y $q_{\text{est}}$ como los cuaterniones de orientación real y estimado, el error se obtiene a partir del cálculo del cuaternión que representa la rotación de uno hacia otro:

$$
q_{\text{err}} = q_{\text{real}} \otimes q_{\text{est}}^*
$$

Utilizando la convención de vectores columna:

$$
q = \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix}
$$

para el caso ideal se tiene que 

$$
q_{\text{err}} = \begin{bmatrix} 1 \\ 0 \\ 0 \\ 0 \end{bmatrix}
$$

Una medida útil del error es el ángulo de rotación asociado a $q_{\text{err}}$. Por teoría se sabe que

$$
\cos\!\left(\frac{\theta}{2}\right) = q_w
$$

de modo que el ángulo de error se puede calcular como:

$$
\theta_{\text{err}} = 2 \arccos\bigl(q_{w,\text{err}}\bigr)
$$

Idealmente $\theta_{\text{err}} = 0$, aunque se considera aceptable una tolerancia $\theta_{\text{err}} < \theta_{\max}$ donde $\theta_{\max} \approx 5^\circ$. Se implementó el esquema de validación de la Figura 1 y se obtuvo el cuaternión de error comparando los valores real y estimado al imponer como condición inicial $q = q_0$, siendo $q_0$ el cuaternión de orientación real en $t=0$. Se observa que $\theta_{\text{err}} < 2.5^\circ \ \forall t$, dentro de los límites aceptables.

<p align="center">
  <img src="ErrorOrientHelmetPoser.png" alt="Figura 3" style="max-width: 100%;"><br>
  <strong>Figura 3:</strong> Gráfico del error de orientación en función del tiempo para un registro del dataset de HelmetPoser. No se realiza optimización de varianzas.
</p>

La relación entre cuaternión y velocidad angular se expresa como:

$$
\dot{q} = \frac{1}{2} \, q \otimes \boldsymbol{\omega}
$$

donde $\boldsymbol{\omega}$ es la velocidad angular en el sistema del IMU. En la Figura 3 se compara la velocidad angular simulada con la real, mostrando excursiones anormalmente altas durante cambios bruscos de orientación.

<p align="center">
  <img src="VelAngularErrorOrient.png" alt="Figura 4" style="max-width: 100%;"><br>
  <strong>Figura 4:</strong> Velocidad angular simulada y real (izquierda) y errores de cuaternión en función del tiempo (derecha)
</p>

## Optimización de varianzas de Ruido

Se estimaron las varianzas de los ruidos del giroscopio, acelerómetro y magnetómetro: $\sigma_a^2, \sigma_g^2, \sigma_m^2$. Esto aplica principalmente al EKF.

1. <em>Random Search</em> con Optuna (https://optuna.org/)  
2. <em>Grid Search</em> personalizado, presentado en la Figura 4

<p align="center">
  <img src="DiagramaIMU-Optim_Varianzas.drawio.png" alt="Figura 5" style="max-width: 100%;"><br>
  <strong>Figura 5:</strong> Esquema del algoritmo de optimización de varianzas usando <em>grid search</em>
</p>

El valor cuadrático medio del error de orientación se define como:

$$
RMS = \sqrt{\sum_{n=1}^{N} \Bigl( 2 \arccos \bigl| q_{w,\text{err},n} \bigr| \Bigr)^2}
$$

<p align="center">
  <img src="ComparacionOrient.png" alt="Figura 6" style="max-width: 100%;"><br>
  <strong>Figura 6:</strong> Comparación del error de orientación antes y después de optimización
</p>

## Análisis Futuro

Para determinar la posición, velocidad y orientación relativa entre dos nodos $i$ y $j$ mediante mediciones de ranging, se pueden plantear dos enfoques:

1. Unificar estimación de orientación y INS en un único EKF:

$$
\mathbf{x} = \begin{bmatrix} q_{ij} \\ v_{ij} \\ p_{ij} \end{bmatrix} \in \mathbb{R}^{10}
$$

2. Calcular orientación relativa externamente y usarla como entrada de EKF reducido:

$$
q_{ij} = q_i \otimes q_j^*, \quad 
\mathbf{x} = \begin{bmatrix} v_{ij} \\ p_{ij} \end{bmatrix} \in \mathbb{R}^{6}
$$

La ecuación de medición basada en ranging es:

$$
z_k = h(x_k) + v_k = \| p_{ij,k} \|_2 + v_k
$$

donde $v_k$ representa ruido AWGN del proceso.