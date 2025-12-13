import pandas as pd
import sys
from ahrs.common.quaternion import *
import numpy as np
from matplotlib import pyplot as plt
from pyquaternion import *
from ahrs.utils.sensors import Sensors
from scipy.constants import *
import scipy.io as sio
from rosbag import *
from Orientacion import *
from Optimizacion import *
from SimuladorIMU import Simulator

## Ruta de la carpeta raíz en donde está el proyecto
## (adaptar al equipo en cuestión antes de usar el código)
ruta_raiz = "C:/Yo/Proyecto IMUs"

## Ejecución principal del programa
if __name__== '__main__':

    ## Opcion 1: Uso de simulador con ground-truth de ejemplo
    ## Opcion 2: Uso de simulador ground-truth de PyShoe
    ## Opcion 3: Uso de simulador con ground-truth de HelmetPoser
    opcion = 1

    ## En caso de que quiera trabajar con un simulador distinto al de AHRS
    if opcion == 1:

        ## Cargo el archivo .csv donde tengo los datos de ground-truth
        csv = np.genfromtxt("GroundTruth.txt", delimiter = "", skip_header = 1)

        ## Obtengo la orientación real en cuaterniones expresados (qw, qx, qy, qz)
        cuat_real = np.array((csv[:, 7], csv[:, 4], csv[:, 5], csv[:, 6])).transpose()

        ## Obtengo la posición real expresada como (tx, ty, tz)
        pos_real = np.array((csv[:, 1], csv[:, 2], csv[:, 3])).transpose()

        ## La entrada <<csv>> debe ser un array bidimensional numpy que tenga 8 columnas
        ## (timestamp, tx, ty, tz, qx, qy, qz, qw) como las columnas en ese orden que debe tener el archivo
        simulator = Simulator(csv, xyzw = True)

        ## Obtengo los valores de aceleración simulada en m/s2
        acel_simulada = g * simulator.accelerometer

        ## Obtengo los valores de velocidad angular simulada en rad/s
        gyro_simulada = np.deg2rad(simulator.gyroscope)

        ## Simulación del IMU mediante la librería AHRS
        sensor = Sensors(quaternions_input = cuat_real, freq = simulator.sample_rate)

        ## Hago el filtrado de la señal para obtener orientacion
        filtro = Filtrado(acel_simulada, gyro_simulada, sensor.magnetometers, modelo = 'ekf', fs = simulator.sample_rate, frame = 'NED', noises = (0.001 ** 2, 0.1 ** 2, 0.1 ** 2))

        ## Obtengo los cuaterniones de offset del ground truth respecto de la estimación
        cuat_offset, cuat_offset_vec = CuaternionesOffset(cuat_real, filtro.Q)

        ## Creación de figura y ejes para la graficación
        figure, axis = plt.subplots(2, 1, figsize = (10, 10))

        ## Graficación del coseno de rotacion entre el sistema ground truth y el calculado
        axis[0].plot(np.abs(cuat_offset_vec[:,0]), color = 'b')
        axis[0].set_xlabel("Número de muestra")
        axis[0].set_ylabel("|cos($\phi$/2)|")
        axis[0].set_title("Error de orientación")

        ## Graficación de la velocidad angular simulada
        axis[1].plot(np.sqrt(np.sum(np.square(gyro_simulada), axis = 1)), color = 'r')
        axis[1].set_xlabel("Número de muestra")
        axis[1].set_ylabel("|$\omega$(rad/s)|")
        axis[1].set_title("Velocidad angular simulada")
        plt.legend()
        plt.show()

        # ## Se realiza la optimización de las varianzas con derivadas parciales (descomentar si se va a optimizar)
        # var_gyro, var_acc, var_mag = OptimizarVarianzas(acel_simulada, gyro_simulada, mag_sintet, simulator.sample_rate, 'NED', cuat_real, 0.1, plot = True, varianza_inicial = [0.001 ** 2, 0.1 ** 2, 0.1 ** 2])

        # ## Hago la optimización usando la librería de Optuna (descomentar si se va a optimizar)
        # var_gyro_opt, var_acc_opt, var_mag_opt = OptimVarianzasOptuna(gyro_simulada, acel_simulada, mag_sintet, 'NED', simulator.sample_rate, cuat_real, 20, [0.001 ** 2, 0.1 ** 2, 0.1 ** 2])

        ## Hago el filtrado de la señal para obtener orientacion con las varianzas optimizadas
        filtro_opt = Filtrado(acel_simulada, gyro_simulada, sensor.magnetometers, modelo = 'ekf', fs = simulator.sample_rate, frame = 'NED', noises = (1.0155995666841593e-10, 2.3041857353477093, 0.04440085932854474))

        ## Obtengo los cuaterniones de offset y la matriz de rotación de offset
        cuat_offset_opt, cuat_offset_vec_opt = CuaternionesOffset(cuat_real, filtro_opt.Q)

        ## Hago el filtrado optimizado con las varianzas de optuna
        filtro_optuna = Filtrado(acel_simulada, gyro_simulada, sensor.magnetometers, modelo = 'ekf', fs = simulator.sample_rate, frame = 'NED', noises = (0.000549210487787499, 1.985197840445696, 0.035499415508008524))

        ## Obtengo los cuaterniones de offset y la matriz de rotación de offset
        cuat_offset_optuna, cuat_offset_vec_optuna = CuaternionesOffset(cuat_real, filtro_optuna.Q)

        ## Gráfica comparativa entre los errores de orientación antes y después de optimizar las varianzas
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec[:,0]))), label = 'Sin Optimizar, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec[:, 0]))))), 3)))
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec_opt[:,0]))), label = 'Optimizado, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec_opt[:, 0]))))), 3)))
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec_optuna[:,0]))), label = 'Optimizado Optuna, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec_optuna[:, 0]))))), 3)))
        plt.xlabel("Número de muestra")
        plt.ylabel("Grados de rotación (°)")
        plt.title("Error de orientación ($\Theta$)")
        plt.legend()
        plt.show()

        ## Hago el filtrado de la señal para obtener orientacion pasando como orientación inicial el cuaternión real
        filtro_init = Filtrado(acel_simulada, gyro_simulada, sensor.magnetometers, modelo = 'ekf', fs = simulator.sample_rate, frame = 'NED', noises = (0.000549210487787499, 1.985197840445696, 0.035499415508008524), q0 = cuat_real[0, :])

        ## Obtengo los cuaterniones de offset y la matriz de rotación de offset
        cuat_offset_init, cuat_offset_vec_init = CuaternionesOffset(cuat_real, filtro_init.Q)

        ## Para evitar errores de redondeo, seteo el cuaternión de offset inicial como el cuaternión identidad [1, 0, 0, 0]
        ## en el caso donde yo configure como orientación inicial aquella igual a la del ground-truth
        cuat_offset_init[0] = Quaternion(1, 0, 0, 0)
        cuat_offset_vec_init[0, :] = Quaternion(1, 0, 0, 0).q

        ## Graficación del ángulo de rotacion entre el sistema ground truth y el calculado
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec_init[:, 0]))), label = 'Estado Inicial Real, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec_init[1:, 0]))))), 3)))
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec_optuna[:, 0]))), label = 'Estado Inicial Arbitrario, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec_optuna[:, 0]))))), 3)))
        plt.xlabel("Número de muestra")
        plt.ylabel("Grados de rotación (°)")
        plt.title("Error de orientación ($\Theta$)")
        plt.legend()
        plt.show()

    ## En caso de que quiera simular con en dataset de PyShoe
    elif opcion == 2:

        ## Especifico identificador de la carpeta en la cual se encuentran los datos de PyShoe
        ident = "2017-11-22-11-25-20"

        ## Cargado del fichero de datos crudos
        datos_crudos = pd.read_csv('{}/Datasets/DatasetPyShoe/data/vicon/raw/{}/imudata.csv'.format(ruta_raiz, ident))

        ## Cargado del fichero de datos procesados
        datos_procesados = sio.loadmat('{}/Datasets/DatasetPyShoe/data/vicon/raw/{}/processed_data.mat'.format(ruta_raiz, ident))

        ## Obtengo las aceleraciones en los tres ejes
        acel_real = np.array([np.array(datos_crudos['x.2'])[1:-1], np.array(datos_crudos['y.2'][1:-1]), np.array(datos_crudos['z.2'])[1:-1]]).transpose()

        ## Obtengo las velocidades angulares en los tres ejes
        gyro_real = np.array([np.array(datos_crudos['x.1'])[1:-1], np.array(datos_crudos['y.1'][1:-1]), np.array(datos_crudos['z.1'])[1:-1]]).transpose()

        ## Obtengo la posición ground-truth en los tres ejes
        pos_real = datos_procesados['gt']

        ## Obtengo la secuencia de los time-stamp
        timestamp_real = datos_procesados['ts']

        ## Obtengo al orientación ground-truth usando cuaterniones
        cuat_real = np.array([np.array(datos_crudos['w'])[1:-1], np.array(datos_crudos['x'])[1:-1], np.array(datos_crudos['y'])[1:-1],
                        np.array(datos_crudos['z'])[1:-1]]).transpose()

        ## Genero una estructura <<csv>> debe ser un array bidimensional numpy que tenga 8 columnas
        ## (timestamp, tx, ty, tz, qx, qy, qz, qw) como las columnas en ese orden
        datos_vicon = np.array([np.reshape(timestamp_real, (timestamp_real.shape[1],)), pos_real[:,0], pos_real[:,1], pos_real[:,2],
                        np.array(datos_crudos['x'])[1:-1], np.array(datos_crudos['y'])[1:-1], np.array(datos_crudos['z'])[1:-1], np.array(datos_crudos['w'])[1:-1]]).transpose()

        ## La entrada <<csv>> debe ser un array bidimensional numpy que tenga 8 columnas
        ## (timestamp, tx, ty, tz, qx, qy, qz, qw) como las columnas en ese orden que debe tener el archivo
        simulator = Simulator(datos_vicon, xyzw = True)

        ## Obtengo los valores de aceleración simulada en m/s2
        acel_simulada = g * simulator.accelerometer

        ## Obtengo los valores de velocidad angular simulada en rad/s
        gyro_simulada = np.deg2rad(simulator.gyroscope)

        ## Simulación del IMU mediante la librería AHRS
        sensor = Sensors(quaternions_input = cuat_real, freq = simulator.sample_rate)

        ## Hago el filtrado de la señal para obtener orientacion
        filtro = Filtrado(acel_simulada, gyro_simulada, sensor.magnetometers, modelo = 'ekf', fs = simulator.sample_rate, frame = 'NED', noises = (0.001 ** 2, 0.1 ** 2, 0.1 ** 2))

        ## Obtengo los cuaterniones de offset del ground truth respecto de la estimación
        cuat_offset, cuat_offset_vec = CuaternionesOffset(cuat_real, filtro.Q)

        ## Graficación de la velocidad angular simulada IMUSim
        plt.subplot(221)
        plt.plot(np.sqrt(np.sum(np.square(sensor.gyroscopes), axis = 1)), color = 'r')
        plt.xlabel("Número de muestra")
        plt.ylabel("|$\omega$(rad/s)|")
        plt.title("Velocidad angular simulada AHRS")

        ## Graficación del coseno de rotacion entre el sistema ground truth y el calculado
        plt.subplot(222)
        plt.plot(np.abs(cuat_offset_vec[:,0]), color = 'b')
        plt.xlabel("Número de muestra")
        plt.ylabel("|cos($\phi$/2)|")
        plt.title("Error de orientación (q)")

        ## Graficación de la velocidad angular real
        plt.subplot(223)
        plt.plot(np.sqrt(np.sum(np.square(gyro_real), axis = 1)), color = 'g')
        plt.xlabel("Número de muestra")
        plt.ylabel("|$\omega$(rad/s)|")
        plt.title("Velocidad angular real")

        ## Graficación del ángulo de rotacion entre el sistema ground truth y el calculado
        plt.subplot(224)
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec[:,0]))))
        plt.xlabel("Número de muestra")
        plt.ylabel("Grados de rotación (°)")
        plt.title("Error de orientación ($\Theta$)")
        plt.legend()
        plt.show()

        # ## Se realiza la optimización de las varianzas con derivadas parciales (descomentar si se va a optimizar)
        # var_gyro, var_acc, var_mag = OptimizarVarianzas(acel_simulada, gyro_simulada, mag_sintet, simulator.sample_rate, 'NED', cuat_real, 0.1, plot = True, varianza_inicial = [0.001 ** 2, 0.1 ** 2, 0.1 ** 2])

        # ## Hago la optimización usando la librería de Optuna (descomentar si se va a optimizar)
        # var_gyro_opt, var_acc_opt, var_mag_opt = OptimVarianzasOptuna(gyro_simulada, acel_simulada, mag_sintet, 'NED', simulator.sample_rate, cuat_real, 20, [0.001 ** 2, 0.1 ** 2, 0.1 ** 2])

        ## Hago el filtrado de la señal para obtener orientacion con las varianzas optimizadas
        filtro_opt = Filtrado(acel_simulada, gyro_simulada, sensor.magnetometers, modelo = 'ekf', fs = simulator.sample_rate, frame = 'NED', noises = (2.892546549759998e-05, 0.016800000000000002, 0.010000000000000002))

        ## Obtengo los cuaterniones de offset y la matriz de rotación de offset
        cuat_offset_opt, cuat_offset_vec_opt = CuaternionesOffset(cuat_real, filtro_opt.Q)

        ## Hago el filtrado optimizado con las varianzas de optuna
        filtro_optuna = Filtrado(acel_simulada, gyro_simulada, sensor.magnetometers, modelo = 'ekf', fs = simulator.sample_rate, frame = 'NED', noises = (0.0989302473886146, 1.8945097900957693, 0.0872458695681404))

        ## Obtengo los cuaterniones de offset y la matriz de rotación de offset
        cuat_offset_optuna, cuat_offset_vec_optuna = CuaternionesOffset(cuat_real, filtro_optuna.Q)

        ## Gráfica comparativa entre los errores de orientación antes y después de optimizar las varianzas
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec[:,0]))), label = 'Sin Optimizar, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec[:, 0]))))), 3)))
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec_opt[:,0]))), label = 'Optimizado, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec_opt[:, 0]))))), 3)))
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec_optuna[:,0]))), label = 'Optimizado Optuna, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec_optuna[:, 0]))))), 3)))
        plt.xlabel("Número de muestra")
        plt.ylabel("Grados de rotación (°)")
        plt.title("Error de orientación ($\Theta$)")
        plt.legend()
        plt.show()

        ## Hago el filtrado de la señal para obtener orientacion pasando como orientación inicial el cuaternión real
        filtro_init = Filtrado(acel_simulada, gyro_simulada, sensor.magnetometers, modelo = 'ekf', fs = simulator.sample_rate, frame = 'NED', noises = (0.0989302473886146, 1.8945097900957693, 0.0872458695681404), q0 = cuat_real[0, :])

        ## Obtengo los cuaterniones de offset y la matriz de rotación de offset
        cuat_offset_init, cuat_offset_vec_init = CuaternionesOffset(cuat_real, filtro_init.Q)

        ## Para evitar errores de redondeo, seteo el cuaternión de offset inicial como el cuaternión identidad [1, 0, 0, 0]
        ## en el caso donde yo configure como orientación inicial aquella igual a la del ground-truth
        cuat_offset_init[0] = Quaternion(1, 0, 0, 0)
        cuat_offset_vec_init[0, :] = Quaternion(1, 0, 0, 0).q

        ## Graficación del ángulo de rotacion entre el sistema ground truth y el calculado
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec_init[:, 0]))), label = 'Estado Inicial Real, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec_init[:, 0]))))), 3)))
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec_optuna[:, 0]))), label = 'Estado Inicial Arbitrario, RMS = {}'.
                format(round(np.sqrt(np.sum(np.square(2 * np.arccos(np.abs(cuat_offset_vec_optuna[:, 0]))))), 3)))
        plt.xlabel("Número de muestra")
        plt.ylabel("Grados de rotación (°)")
        plt.title("Error de orientación ($\Theta$)")
        plt.legend()
        plt.show()
    
    ## En caso de que quiera simular el dataset de HelmetPoser
    elif opcion == 3:

        ## Lectura de archivo bag
        bagfile = Bag('{}/Datasets/HelmetPoser/C_r.bag'.format(ruta_raiz))

        ## Construyo un vector vacío para guardar los valores de ground-truth en cada caso
        acel_real = []
        gyro_real = []
        cuat_real = []

        ## Hago la lectura en el archivo .bag para obtener la información de ground-truth
        for topic, msg, t in bagfile.read_messages(topics = ['/vectornav/IMU']):

            ## Agrego las aceleraciones al vector correspondiente
            acel_real.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

            ## Agrego las velocidades angulares al vector correspondiente
            gyro_real.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

            ## Agrego los cuaterniones de orientación al vector correspondiente
            cuat_real.append([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])

        ## Conversión a matrices de tipo numpy array para los datos de ground-truth
        acel_real = np.array(acel_real)
        gyro_real = np.array(gyro_real)
        cuat_real = np.array(cuat_real)

        ## Simulación del IMU mediante la librería AHRS (frecuencia 200Hz es dato del paper)
        sensor = Sensors(quaternions_input = cuat_real, freq = 200)

        ## Hago el filtrado de la señal para obtener orientacion (frecuencia 200Hz es dato del paper)
        filtro = Filtrado(sensor.accelerometers, sensor.gyroscopes, sensor.magnetometers, modelo = 'ekf', fs = 200, frame = 'NED', noises = (0.0989302473886146, 1.8945097900957693, 0.0872458695681404), q0 = cuat_real[0, :])

        ## Obtengo los cuaterniones de offset del ground truth respecto de la estimación
        cuat_offset, cuat_offset_vec = CuaternionesOffset(cuat_real, filtro.Q)

        ## Graficación del coseno de rotacion entre el sistema ground truth y el calculado
        plt.plot(np.rad2deg(2 * np.arccos(np.abs(cuat_offset_vec[:,0]))))
        plt.xlabel("Número de muestra")
        plt.ylabel("Error de Orientación")
        plt.legend()
        plt.show()

        ## Graficación de la velocidad angular real con respecto a la velocidad angular simulada
        plt.plot(np.sqrt(np.sum(np.square(gyro_real), axis = 1)), color = 'r', label = 'Velocidad angular real')
        plt.plot(np.sqrt(np.sum(np.square(sensor.gyroscopes), axis = 1)), color = 'b', label = 'Velocidad angular simulada')
        plt.xlabel("Número de muestra")
        plt.ylabel("|$\omega$(rad/s)|")
        plt.legend()
        plt.show()