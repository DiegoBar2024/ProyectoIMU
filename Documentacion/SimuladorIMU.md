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