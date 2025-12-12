import ahrs
from ahrs.common.quaternion import *
import numpy as np
from matplotlib import pyplot as plt
from pyquaternion import *
from scipy.constants import *
from optuna import *
from Orientacion import CuaternionesOffset, Filtrado

## Función que me haga la optimización de las varianzas del acelerómetro, giroscopio y magnetómetro
def OptimVarianzasOptuna(gyro, acel, mag, frame, fs, cuat_real, iteraciones, noises, plot = True):

    """
    Parameters
    ----------
    gyro: numpy.ndarray
        N-by-3 array with measurements of angular velocity in rad/s
    acel: numpy.ndarray
        N-by-3 array with measurements of acceleration in in m/s^2
    mag: numpy.ndarray
        N-by-3 array with measurements of magnetic field in nT
    frame: string
        Inertial frame used in the attitude estimation. Possible values are ['ENU', 'NED']
    fs: float
        Sampling frequency in Hertz.
    cuat_real: numpy.ndarray
        N-by-4 array with the ground truth attitude quaternion sequence
    iteraciones: int
        Number of iterations for the optimization algorithm
    noises: numpy.ndarray
        1-by-3 array containing noise variances in the order [gyroscope, accelerometer, magnetometer]
    plot: bool
        Boolean value indicating if the user wants the results to be plotted

    Returns
    ----------
    var: tuple
        Tuple containing the optimized noise variances in the order [gyroscope, accelerometer, magnetometer]
    """

    ## Creo la configuriación como la función objetivo para optimizar
    func_obj = lambda trial: ObjetivoVarianzasOptuna(trial, gyro, acel, mag, frame, fs, cuat_real)

    ## Construcción de un estudio de optimización en base al muestreador que me minimice
    study = create_study(directions = ["minimize"])

    ## Especifico condiciones iniciales
    study.enqueue_trial({"vag_ang": noises[0], "vag_acc": noises[1], "vag_mag": noises[2]})
    
    ## Hago la optimización de los parámetros
    study.optimize(func_obj, n_trials = iteraciones)

    ## Construyo una lista donde me guardo los valores de la función de costo para cada trial
    loss = []

    ## Construyo una lista donde me guardo los valores de las varianzas en los tres ejes para cada trial
    varianzas = []

    ## Itero para cada una de las pruebas realizadas
    for i in range (iteraciones):

        ## Me guardo el valor de la función costo la i-ésima prueba
        loss.append(study.trials[i].value)

        ## Me guardo el valor de las varianzas para la i-ésima prueba
        varianzas.append([study.trials[i].params['var_ang'], study.trials[i].params['var_acc'], study.trials[i].params['var_mag']])
    
    ## En caso de que quiera graficar el costo en función de la iteración
    if plot:

        ## Configuro parámetros descriptivos del gráfico
        plt.plot(loss)
        plt.xlabel("Número de iteración")
        plt.ylabel("Error orientación (°)")
        plt.legend()
        plt.show()

    ## Retorno los valores optimizados de varianzas de giroscopio, acelerómetro y magnetómetro
    return study.best_trials[0].params['var_ang'], study.best_trials[0].params['var_acc'], study.best_trials[0].params['var_mag']

## Función que implementa la función objetivo de la optimización de las varianzas
def ObjetivoVarianzasOptuna(trial, gyro, acel, mag, frame, fs, cuat_real):

    """
    Parameters
    ----------
    gyro: numpy.ndarray
        N-by-3 array with measurements of angular velocity in rad/s
    acel: numpy.ndarray
        N-by-3 array with measurements of acceleration in in m/s^2
    mag: numpy.ndarray
        N-by-3 array with measurements of magnetic field in nT
    frame: string
        Inertial frame used in the attitude estimation. Possible values are ['ENU', 'NED']
    fs: float
        Sampling frequency in Hertz.
    cuat_real: numpy.ndarray
        N-by-4 array with the ground truth attitude quaternion sequence
    
    Returns
    ----------
    loss: float
        Loss value of the iteration
    """

    ## Rango de las varianzas del giroscopio
    var_ang = trial.suggest_float("var_ang", 0, 0.1)

    ## Rango de las varianzas del acelerómetro
    var_acc = trial.suggest_float("var_acc", 0, 2)

    ## Rango de las varianzas del magnetómetro
    var_mag = trial.suggest_float("var_mag", 0, 0.1)

    ## Hago el cálculo del filtrado
    filtro = ahrs.filters.EKF(gyr = gyro, acc = acel, mag = mag, frame = frame, frequency = fs, noises = [var_ang, var_acc, var_mag])

    ## Obtengo los cuaterniones de error de orientación con respecto a los cuaterniones ground-truth
    cuat_offset, cuat_offset_vec = CuaternionesOffset(cuat_real, filtro.Q)

    ## Obtengo la función de costo como el valor cuadrático medio de los ángulos de error
    loss = np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec[:, 0])))))

    ## Retorno el valor de la función de costo
    return loss

## Función que grafique el error de orientación a medida que modifico el valor de una de las varianzas
def ModificarVarianzas(tipo, acel, gyro, mag, fs, frame, cuat_real, noises, plot = True):

    """
    Parameters
    ----------
    tipo: string
        Type of noise variance being opimized. Possible values: ['gyro', 'acel', 'mag']
    acel: numpy.ndarray
        N-by-3 array with measurements of acceleration in in m/s^2
    gyro: numpy.ndarray
        N-by-3 array with measurements of angular velocity in rad/s
    mag: numpy.ndarray
        N-by-3 array with measurements of magnetic field in nT
    fs: float
        Sampling frequency in Hertz.
    frame: string
        Inertial frame used in the attitude estimation. Possible values are ['ENU', 'NED']
    cuat_real: numpy.ndarray
        N-by-4 array with the ground truth attitude quaternion sequence
    noises: numpy.ndarray
        1-by-3 array containing the initial noise variances in the order [gyroscope, accelerometer, magnetometer]
    plot: bool
        Boolean value indicating if the user wants the results to be plotted
    
    Returns
    ----------
    var_min: float
        Minimum noise variance value in the iteration. Equivalently, the noise variance value associated to the minimum loss value
    loss_min: float
        Minimum loss value in the iteration
    """

    ## Construyo una lista vacía donde me guardo los valores del RMS del error
    losses = np.zeros(5)

    ## Creo una lista donde me voy guardando las varianzas
    varianzas = np.zeros(5)

    ## Itero para cada uno de los valores de las varianzas obtenidos
    for i in range (-2, 3):

        ## En caso de que varíe el ruido de giroscopio
        if tipo == 'gyro':

            ## Defino la varianza de entrada en este caso
            varianza = (1 + i / 5) * noises[0]

            ## Hago el filtrado usando el valor i-ésimo de la varianza
            filtro = Filtrado(acel, gyro, mag, modelo = 'ekf', fs = fs, frame = frame, noises = [varianza, noises[1], noises[2]])

            ## Seteo el título de la grafica
            titulo = "Varianza Giroscopio. $\sigma_a^2$ = {}, $\sigma_m^2$ = {}".format(noises[1], noises[2])

        ## En caso de que varíe el ruido del acelerómetro
        elif tipo == 'acel':

            ## Defino la varianza de entrada en este caso
            varianza = (1 + i / 5) * noises[1]

            ## Hago el filtrado usando el valor i-ésimo de la varianza
            filtro = Filtrado(acel, gyro, mag, modelo = 'ekf', fs = fs, frame = frame, noises = [noises[0], varianza, noises[2]])

            ## Seteo el título de la grafica
            titulo = "Varianza Acelerómetro. $\sigma_g^2$ = {}, $\sigma_m^2$ = {}".format(noises[0], noises[2])
        
        ## En caso de que varíe el ruido del magnetómetro
        elif tipo == 'mag':

            ## Defino la varianza de entrada en este caso
            varianza = (1 + i / 5) * noises[2]

            ## Hago el filtrado usando el valor i-ésimo de la varianza
            filtro = Filtrado(acel, gyro, mag, modelo = 'ekf', fs = fs, frame = frame, noises = [noises[0], noises[1], varianza])

            ## Seteo el título de la grafica
            titulo = "Varianza Magnetómetro. $\sigma_g^2$ = {}, $\sigma_m^2$ = {}".format(noises[0], noises[1])
        
        ## Obtengo los cuaterniones de offset y la matriz de rotación de offset
        cuat_offset, cuat_offset_vec = CuaternionesOffset(cuat_real, filtro.Q)

        ## Obtengo una métrica de error de orientación
        loss = np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec[:, 0])))))

        ## Agrego la varianza a la lista correspondiente
        varianzas[i] = varianza

        ## Agrego el valor RMS del error a la lista correspondiente
        losses[i] = loss

        ## En caso que quiera graficar
        if plot:

            ## Hago la graficación de los errores
            plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec[:,0]))), label = "$\sigma^2$ = {}, Loss = {}".format(varianza, round(loss, 2)))

    ## En caso de que quiera graficar el costo en función de la iteración
    if plot:

        ## Configuro parámetros generales del gráfico
        plt.xlabel("Número de iteración")
        plt.ylabel("Error orientación (°)")
        plt.title(titulo)
        plt.legend()
        plt.show()
    
    ## Retorno la varianza óptima y el correspondiente valor RMS
    return varianzas[np.argmin(losses)], np.min(losses)

## Función para hacer la optimización de las varianzas haciendo grid search
def OptimizarVarianzas(acel, gyro, mag, fs, frame, cuat_real, tol, plot = True, varianza_inicial = [0.1, 0.1, 0.1]):

    """
    Parameters
    ----------
    acel: numpy.ndarray
        N-by-3 array with measurements of acceleration in in m/s^2
    gyro: numpy.ndarray
        N-by-3 array with measurements of angular velocity in rad/s
    mag: numpy.ndarray
        N-by-3 array with measurements of magnetic field in nT
    fs: float
        Sampling frequency in Hertz.
    frame: string
        Inertial frame used in the attitude estimation. Possible values are ['ENU', 'NED']
    cuat_real: numpy.ndarray
        N-by-4 array with the ground truth attitude quaternion sequence
    tol: float
        Absolute error tolerance value in the optimization algorithm
    plot: bool
        Boolean value indicating if the user wants the results to be plotted

    Returns
    ----------
    var: tuple
        Tuple containing the optimized noise variances in the order [gyroscope, accelerometer, magnetometer]
    """

    ## Configuro condiciones iniciales
    [var_gyro_min, var_acc_min, var_mag_min] = varianza_inicial

    ## Hago el filtrado usando los valores de varianza iniciales
    filtro = Filtrado(acel, gyro, mag, modelo = 'ekf', fs = fs, frame = frame, noises = varianza_inicial)

    ## Obtengo los cuaterniones de offset y la matriz de rotación de offset
    cuat_offset, cuat_offset_vec = CuaternionesOffset(cuat_real, filtro.Q)

    ## Construyo una lista vacía donde voy a guardar los errores
    losses = []

    ## Hago el cálculo del error inicial la guardo en la lista
    losses.append(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec[:, 0]))))))

    ## Mientras el valor absoluto del error sea menor a una determinada tolerancia
    while True:

        ## Impresión de mensaje en la consola
        print("Número de iteración: {}".format(len(losses)))

        ## Optimizo la varianza del giroscopio
        var_gyro_min, loss_gyro_min = ModificarVarianzas('gyro', acel, gyro, mag, fs, frame, cuat_real, [var_gyro_min, var_acc_min, var_mag_min], False)

        ## Optimizo la varianza del acelerómetro
        var_acc_min, loss_acc_min = ModificarVarianzas('acel', acel, gyro, mag, fs, frame, cuat_real, [var_gyro_min, var_acc_min, var_mag_min], False)

        ## Optimizo la varianza del magnetómetro    
        var_mag_min, loss_mag_min = ModificarVarianzas('mag', acel, gyro, mag, fs, frame, cuat_real, [var_gyro_min, var_acc_min, var_mag_min], False)

        ## Agrego el error óptimo de la iteración a la lista
        losses.append(loss_mag_min)

        ## En caso de que la diferencia absoluta entre las últimas dos medidas sea menor a la tolerancia
        if np.abs(losses[-1] - losses[-2]) < tol and len(losses) > 10:

            ## Finalizo la ejecución del bucle
            break

    ## En caso de que quiera graficar la evolución del costo en el tiempo
    if plot:

        ## Parámetros de graficación
        plt.plot(losses)
        plt.xlabel("Número de iteración")
        plt.ylabel("RMS de Error de Orientación")
        plt.legend()
        plt.show()
    
    ## Retorno los valores óptimos de las varianzas calculados con grid search
    return var_gyro_min, var_acc_min, var_mag_min