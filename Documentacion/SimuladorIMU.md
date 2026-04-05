# Simulador IMU

## Modelos de acelerómetro y giroscopio

Para desarrollar el simulador de IMU en principio se utilizan los modelos convencionales de acelerómetro y giroscopio. Se denotan $n$ y $b$ como el sistema solidario a la Tierra y el sistema del IMU respectivamente. El modelo del giroscopio está dado por:

$$ 
y_{\omega} = \omega + b_{\omega} + n_{\omega}
$$

En la ecuación anterior, $y_{\omega}$ representa la medición del giroscopio, $b_{\omega}$ es el bias del giroscopio, $n_{\omega}$ es el ruido asociado a la medición del giroscopio, y $\omega$ es la velocidad angular real expresada en $b$.

El modelo del acelerómetro está dado por:

$$
y_{a} = R_{n}^{b}(a - g) + b_{a} + n_{a}
$$

En la ecuación anterior, $y_{a}$ representa la medición del acelerómetro, $b_{a}$ es el bias del acelerómetro, $n_{a}$ es el ruido asociado a la medición del acelerómetro, $a$ es la aceleración traslacional en el sistema inercial $n$, $g$ es la aceleración gravitatoria en el sistema inercial $n$, $R_{n}^{b}$ es la matriz de rotación del sistema inercial $n$ al sistema del IMU $b$.

Una hipótesis usual es considerar a los términos de ruido estocástico $n_{\omega}$ y $n_{a}$ como procesos blancos gaussianos de media nula de manera que: 

$$
n_{\omega} \sim \mathcal{N}(0,\Sigma_{\omega})
$$

$$
n_{a}  \sim \mathcal{N}(0,\Sigma_{a})
$$

Una hipótesis aún más restrictiva implica considerar que las distintas componentes de ruido son independientes y tienen la misma varianza. En ese caso las matrices de covarianza se simplifican a:

$$
\Sigma_{\omega} = \sigma_{\omega}^{2}I_{3}
$$

$$
\Sigma_{a} = \sigma_{a}^{2}I_{3}
$$

siendo $\sigma_{\omega}^{2}$ y $\sigma_{a}^{2}$ las varianzas asociadas a los ruidos de giroscopio y acelerómetro respectivamente, y $I_{3}$ la matriz identidad de dimensión 3.

## Modelos de bias

Es común modelar los bias del acelerómetro y giroscopio como $\textit{random walks}$ de manera que las ecuaciones de la dinámica de los bias en tiempo contínuo están dadas por:

$$
\dot{b_{a}}=n_{b_{a}}
$$

$$
\dot{b_{\omega}}=n_{b_{\omega}}
$$

Al igual que en los modelos del acelerómetro y giroscopio, los términos de ruido estocástico $n_{b_{a}}$ y $n_{b_{\omega}}$ asociados a los bias se modelan como procesos AWGN de modo que:

$$
n_{b_{a}}  \sim \mathcal{N}(0,\Sigma_{b_{a}})
$$

$$
n_{b_{\omega}} \sim \mathcal{N}(0,\Sigma_{b_{\omega}})
$$

Al hacer la discretización de ambas ecuaciones de los bias se obtiene lo siguiente:

$$
b_{a_{k+1}} = b_{a_{k}} + n_{b_{a_{k}}}
$$

$$
b_{\omega_{k+1}} = b_{\omega_{k}} + n_{b_{\omega_{k}}}
$$

## Ecuaciones del simulador

Se supone un escenario en el cual se conoce la posición $p$ y orientación $q$ del sistema del IMU $b$ relativa al sistema inercial $n$. Sea $T$ el período de muestreo del sistema. Es posible calcular la aceleración traslacional en el sistema $n$ mediante una discretización de la segunda derivada:

$$
\tilde{a_{k}} = \frac{p_{k-1} - 2p_{k} + p_{k+1}}{T^{2}}
$$

Antes de comenzar compruebo que los cuaterniones estén normalizados de modo que $\lVert q \rVert=1$. Por otro lado, dado que los cuaterniones unitarios $q$ y $-q$ representan la misma rotación, efectúo una corrección de signo previo al procesamiento de los cuaterniones.

Dado que los cuaterniones y las matrices de rotación son equivalentes, puedo utilizar $q$ para calcular $R_{n}^{b}$. Se realiza la rotación del sistema inercial $n$ al sistema del IMU $b$ de la aceleración total (traslacional sumada a la gravitatoria):

$$
a_{k} = R_{n}^{b_{k}}(\tilde{a_{k}} - g)
$$

Para poder estimar la medición del giroscopio, primero implemento una variante de la diferencia central de segundo orden para las matrices de rotación. Recordamos que a partir del cuaternión unitario $q$ es posible calcular la matriz de rotación asociada de modo que $R = R(q)$. La rotación producida entre el instante anterior $i-1$ y el instante siguiente $i+1$ está dada por:

$$
R_{\Delta}=R_{i-1}^{-1}R_{i+1}
$$

Es posible estimar el valor de la velocidad angular real a partir del cociente entre el desplazamiento angular $\Delta \theta$ y el período de muestreo $T$, suponiendo que este último es lo suficientemente pequeño. En mi caso, voy a asumir que $\Delta \theta$ es el vector de rotación asociado a la matriz de rotación $R_{\Delta}$, de modo que 

$$
\text{Log}(R_{\Delta}) = \Delta \theta
$$

siendo $\text{Log}:SO(3) \rightarrow \mathbb{R}^{3}$ la operación $\textit{mapa logarítmico}$. Dado que $\Delta \theta$ representa la rotación entre $i-1$ y $i+1$ entonces es posible estimar la velocidad angular real de la forma siguiente:

$$
\omega_{k} \approx \frac{\Delta \theta_{k}}{2T}
$$

asumiendo que el período de muestreo $T$ es lo suficientemente pequeño como para que la aproximación sea válida.

En línea con los modelos de acelerómetro y giroscopio utilizados, se realiza la adición de los términos de bias (modelado como $\textit{random walk}$ y ruido blanco gaussiano) obteniendo las mediciones sintéticas del acelerómetro y el giroscopio. De este modo:

$$
y_{a_{k}} = a_{k} + b_{a_{k}} + n_{a_{k}}
$$

$$
y_{\omega_{k}} = \omega_{k} + b_{\omega_{k}} + n_{\omega_{k}}
$$

siendo $y_{a_{k}}$, $y_{\omega_{k}}$ las mediciones sintéticas de la IMU ficticia.