import ahrs
from ahrs.common.quaternion import *
import numpy as np
from pyquaternion import *
from scipy.constants import *
from optuna import *

## Función que toma como entrada los valores de aceleración, velocidad angular e intensidad de campo magnético
## Se devuelve a la salida el objeto filtro dependiendo del modelo elegido
def Filtrado(acel, gyro, mag, modelo, fs, frame = 'NED', noises = [0.001 ** 2, 0.1 ** 2, 0.1 ** 2], q0 = None):

    """
    Parameters
    ----------
    acel: numpy.ndarray
        N-by-3 array with measurements of acceleration in in m/s^2
    gyro: numpy.ndarray
        N-by-3 array with measurements of angular velocity in rad/s
    mag: numpy.ndarray
        N-by-3 array with measurements of magnetic field in nT
    modelo: string
        Name of the filter model used for attitude estimation. Possible values are ['ekf', 'mahony', 'madgwick', 'complementary']
    fs: float
        Sampling frequency in Hertz.
    frame: string
        Inertial frame used in the attitude estimation. Possible values are ['ENU', 'NED']
    noises: numpy.ndarray
        1-by-3 array containing noise variances in the order [gyroscope, accelerometer, magnetometer]
    q0: numpy.ndarray
        Initial orientation, as a versor (normalized quaternion)

    Returns
    ----------
    filtro: ahrs.filters
        Filter object containing the estimated quaternion attitude sequence
    """

    ## Método I: Filtro de Kalman Extendido
    if modelo == 'ekf':

        ## Procesamiento con EKF (Filtro de Kalman Extendido)
        ## El orden de las varianzas de los ruidos es: giroscopio, acelerómetro, magnetómetro
        filtro = ahrs.filters.EKF(gyr = gyro, acc = acel, mag = mag, frame = frame, frequency = fs, noises = noises, q0 = q0)

    ## Método II: Filtro de Mahony
    elif modelo == 'mahony':

        ## Procesamiento con Mahony
        filtro = ahrs.filters.Mahony(acc = acel, gyr = gyro, mag = mag, frequency = fs)

    ## Método III: Filtro de Madgwick
    elif modelo == 'madgwick':

        ## Procesamiento con Madgwick
        filtro = ahrs.filters.Madgwick(gyr = gyro, acc = acel, mag = mag, frequency = fs, gain = 0.033)

    ## Método IV: Filtro Complementario
    elif modelo == "complementary":

        ## Procesamiento con complementario
        filtro = ahrs.filters.Complementary(gyr = gyro, acc = acel, mag = mag, frequency = fs, gain = 0.9)

    ## Retorno el filtro correspondiente
    return filtro

## Función toma las señales de aceleración y cuaterniones de rotación
## Se hace la rotación del sistema solidario al IMU al sistema inercial
def Rotacion(acel, quat_rotacion):

    """
    Parameters
    ----------
    acel: numpy.ndarray
        N-by-3 array with measurements of acceleration in in m/s^2
    quat_rotacion: numpy.ndarray
        N-by-4 array with the attitude quaternion sequence

    Returns
    ----------
    acel_rotada: numpy.ndarray
        N-by-3 array with acceleration values expressed on inertial frame (in m/s^2)
    """

    ## Construyo una lista en donde voy a almacenar las aceleraciones rotadas
    acel_rotada = []

    ## Itero para cada uno de los cuaterniones del arreglo
    for i in range (quat_rotacion.shape[0]):

        ## Inicializo un objeto conteniendo el cuaternión de rotación
        cuaternion_rotacion = Quaternion(quat_rotacion[i, :][0], quat_rotacion[i, :][1], quat_rotacion[i, :][2], quat_rotacion[i, :][3])

        ## Rotación del vector de aceleración del sistema de la IMU al sistema de navegación
        acel_rotada.append(cuaternion_rotacion.rotate(acel[i, :]))

    ## Retorno la aceleración rotada
    return np.array(acel_rotada)

## Función que me devuelve los cuaterniones de offset entre el cuaternión generado y el groundtruth
def CuaternionesOffset(cuat_truth, cuat_gen):

    """
    Parameters
    ----------
    cuat_truth: numpy.ndarray
        N-by-4 array with the ground truth attitude quaternion sequence
    cuat_gen: numpy.ndarray
        N-by-4 array with the estimated attitude quaternion sequence

    Returns
    ----------
    cuat_offset: numpy.ndarray
        N-by-4 array with offset quaternion sequence. Each quaternion element of the array is expressed as a Quaternion object
    cuat_offset_vec: numpy.ndarray
        N-by-4 array with offset quaternion sequence. Each quaternion element of the array is expressed as a numpy.ndarray object
    """

    ## Creo una lista donde voy a guardar los cuaterniones de offset con el tipo Quaternion
    cuat_offset = []

    ## Inicializo una lista donde guardo los cuaterniones de offset con el tipo numpy array
    cuat_offset_vec = []

    ## Itero para cada uno de los cuaterniones que tengo
    for i in range (cuat_truth.shape[0]):
    
        ## Construyo el cuaternión del ground truth
        cuat_truth_i = Quaternion(cuat_truth[i, :][0], cuat_truth[i, :][1], cuat_truth[i, :][2], cuat_truth[i, :][3])

        ## Construyo el cuaternión estimado
        cuat_gen_i = Quaternion(cuat_gen[i, :][0], cuat_gen[i, :][1], cuat_gen[i, :][2], cuat_gen[i, :][3])

        ## Hago el producto de un cuaternión por el conjugado del otro (me da la orientación relativa) y lo guardo en la lista
        cuat_offset.append(cuat_truth_i * cuat_gen_i.conjugate)

        ## Almaceno cuaternión de offset como vector numpy
        cuat_offset_vec.append(cuat_offset[i].q)

    ## Retorno a la salida los cuaterniones de offset
    return np.array(cuat_offset), np.array(cuat_offset_vec)